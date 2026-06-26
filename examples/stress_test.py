"""Gated faster joint-motion stress test for one RobStride motor.

This test is intended after read, enable, motion, bench, and spin tests have
already passed. It commands larger sine sweeps around the current position,
logs target/feedback to CSV, and disables the motor in a finally block.

It does not write parameters, set zero, change ID, or send firmware commands.
"""

import argparse
import csv
import math
import sys
import time
from pathlib import Path
from typing import Any

from actuator import (
    CH341TransportWrapper,
    RobstrideActuator,
    RobstrideActuatorCommand,
    RobstrideActuatorConfig,
)


def _format_value(value: float | None, unit: str) -> str:
    if value is None:
        return "None"
    return f"{value:.6f} {unit}"


def _format_state(state: Any) -> str:
    return (
        f"id={state.actuator_id} online={state.online} "
        f"pos={_format_value(state.position, 'rad')} "
        f"vel={_format_value(state.velocity, 'rad/s')} "
        f"torque={_format_value(state.torque, 'Nm')} "
        f"temp={_format_value(state.temperature, 'C')}"
    )


def _parse_stages(value: str) -> list[tuple[float, float]]:
    stages = []
    for part in value.split(","):
        part = part.strip()
        if not part:
            continue
        amplitude_text, frequency_text = part.split(":", 1)
        stages.append((float(amplitude_text), float(frequency_text)))
    if not stages:
        raise argparse.ArgumentTypeError("expected stages like 1.0:1.0,2.1:0.75")
    return stages


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _unwrap_position(wrapped_position: float, previous_unwrapped: float) -> float:
    delta = wrapped_position - _wrap_pi(previous_unwrapped)
    if delta > math.pi:
        delta -= 2.0 * math.pi
    elif delta < -math.pi:
        delta += 2.0 * math.pi
    return previous_unwrapped + delta


def _read_state(supervisor: Any, motor_id: int, settle_seconds: float) -> Any | None:
    supervisor.request_feedback(motor_id)
    time.sleep(settle_seconds)
    states = supervisor.get_actuators_state([motor_id])
    return states[0] if states else None


def _require_state(
    supervisor: Any,
    motor_id: int,
    *,
    timeout_seconds: float,
    poll_seconds: float,
) -> Any:
    deadline = time.monotonic() + timeout_seconds
    state = None
    while time.monotonic() < deadline:
        state = _read_state(supervisor, motor_id, poll_seconds)
        if state is not None and state.position is not None:
            return state
    if state is None:
        raise RuntimeError(
            f"missing motor feedback after {timeout_seconds:.3f}s"
        )
    if state.position is None:
        raise RuntimeError(
            f"missing position feedback after {timeout_seconds:.3f}s"
        )
    return state


def _send_position(
    supervisor: Any,
    motor_id: int,
    position: float,
    velocity: float,
    kp: float,
    kd: float,
) -> None:
    cmd = RobstrideActuatorCommand(motor_id)
    cmd.position = position
    cmd.velocity = velocity
    cmd.torque = 0.0
    if not supervisor.command_actuator_now(cmd, kp, kd, True):
        raise RuntimeError("motion command was not sent")


def _enable(supervisor: Any, motor_id: int) -> None:
    print("Sending ENABLE")
    if not supervisor.enable_actuator(motor_id, True):
        raise RuntimeError("enable command was not sent")


def _disable(supervisor: Any, motor_id: int) -> None:
    print("Sending DISABLE")
    disabled = supervisor.disable_actuator(motor_id, False)
    print(f"Disable sent: {disabled}")


def _return_to_start(
    supervisor: Any,
    motor_id: int,
    *,
    start_position: float,
    current_position: float,
    kp: float,
    kd: float,
    seconds: float,
    rate_hz: float,
) -> None:
    steps = max(3, int(seconds * rate_hz))
    for step in range(steps + 1):
        alpha = step / steps
        target = current_position + (start_position - current_position) * alpha
        _send_position(supervisor, motor_id, target, 0.0, kp, kd)
        time.sleep(1.0 / rate_hz)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port-name", required=True)
    parser.add_argument("--motor-id", type=int, required=True)
    parser.add_argument("--motor-type", type=int, required=True)
    parser.add_argument(
        "--stages",
        type=_parse_stages,
        default=[(1.0, 1.0), (2.1, 0.75), (3.14, 0.60)],
        help="comma list amplitude_rad:frequency_hz, e.g. 1.0:1.0,2.1:0.75",
    )
    parser.add_argument("--cycles", type=float, default=3.0)
    parser.add_argument("--kp", type=float, default=60.0)
    parser.add_argument("--kd", type=float, default=2.0)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--csv-file", default="")
    parser.add_argument("--max-pre-velocity", type=float, default=0.5)
    parser.add_argument("--max-velocity", type=float, default=16.0)
    parser.add_argument("--max-feedback-torque", type=float, default=25.0)
    parser.add_argument("--max-temperature", type=float, default=70.0)
    parser.add_argument("--max-tracking-error", type=float, default=2.0)
    parser.add_argument("--feedback-timeout", type=float, default=0.25)
    parser.add_argument("--feedback-poll", type=float, default=0.03)
    parser.add_argument("--dangerous-enable", action="store_true")
    parser.add_argument("--dangerous-motion", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()

    if (
        not args.dangerous_enable
        or not args.dangerous_motion
        or args.confirm != "STRESS"
    ):
        raise SystemExit(
            "Refusing stress test. Re-run with: "
            "--dangerous-enable --dangerous-motion --confirm STRESS"
        )

    for amplitude, frequency_hz in args.stages:
        if not math.isfinite(amplitude) or amplitude <= 0.0:
            raise SystemExit("all stage amplitudes must be finite and > 0")
        if amplitude > math.pi:
            raise SystemExit("stage amplitudes must be <= pi rad")
        if not math.isfinite(frequency_hz) or frequency_hz <= 0.0:
            raise SystemExit("all stage frequencies must be finite and > 0")
        if frequency_hz > 1.25:
            raise SystemExit("stage frequencies must be <= 1.25 Hz")
        peak_target_velocity = amplitude * 2.0 * math.pi * frequency_hz
        if peak_target_velocity > args.max_velocity:
            raise SystemExit(
                "stage peak target velocity exceeds --max-velocity: "
                f"{peak_target_velocity:.3f} rad/s"
            )
    if args.cycles <= 0.0 or args.cycles > 10.0:
        raise SystemExit("--cycles must be > 0 and <= 10")
    if args.kp < 0.0 or args.kp > 120.0:
        raise SystemExit("--kp must be between 0 and 120")
    if args.kd < 0.0 or args.kd > 12.0:
        raise SystemExit("--kd must be between 0 and 12")
    if args.rate_hz < 10.0 or args.rate_hz > 120.0:
        raise SystemExit("--rate-hz must be between 10 and 120")
    if args.max_tracking_error <= 0.0 or args.max_tracking_error > 3.0:
        raise SystemExit("--max-tracking-error must be > 0 and <= 3.0 rad")
    if args.feedback_timeout <= 0.0 or args.feedback_timeout > 1.0:
        raise SystemExit("--feedback-timeout must be > 0 and <= 1.0")
    if args.feedback_poll <= 0.0 or args.feedback_poll > args.feedback_timeout:
        raise SystemExit("--feedback-poll must be > 0 and <= --feedback-timeout")

    csv_path = (
        Path(args.csv_file)
        if args.csv_file
        else Path(
            f"robstride_stress_type{args.motor_type}_id{args.motor_id}_"
            f"{time.strftime('%Y%m%d_%H%M%S')}.csv"
        )
    )

    transport = CH341TransportWrapper(args.port_name)
    supervisor = RobstrideActuator(
        transports=[transport],
        py_actuators_config=[(args.motor_id, RobstrideActuatorConfig(args.motor_type))],
    )

    discovered = supervisor.get_discovered_ids()
    print(f"Discovered motor IDs: {discovered}")
    if args.motor_id not in discovered:
        raise SystemExit(f"Motor ID {args.motor_id} was not discovered")

    print("Pre-stress feedback:")
    state = None
    for _ in range(5):
        state = _require_state(
            supervisor,
            args.motor_id,
            timeout_seconds=args.feedback_timeout,
            poll_seconds=args.feedback_poll,
        )
        print(f"  {_format_state(state)}")
        if state.velocity is not None and abs(state.velocity) > args.max_pre_velocity:
            raise SystemExit(
                f"Refusing stress test: motor is already moving "
                f"{state.velocity:.6f} rad/s"
            )
        time.sleep(0.1)

    start_position = state.position
    if start_position is None:
        raise SystemExit("No start position feedback")

    print(
        "Stress plan: "
        f"start={start_position:+.6f} rad stages={args.stages} "
        f"cycles={args.cycles} kp={args.kp} kd={args.kd}"
    )
    print(f"CSV log: {csv_path}")
    print("Keep clear of the output shaft and be ready to cut motor power.")

    fieldnames = [
        "time_s",
        "amplitude_rad",
        "frequency_hz",
        "target_position_rad",
        "target_velocity_rad_s",
        "actuator_id",
        "online",
        "position_rad",
        "position_unwrapped_rad",
        "velocity_rad_s",
        "torque_nm",
        "temperature_c",
        "position_error_rad",
    ]
    enabled = False
    run_start = time.monotonic()
    previous_unwrapped = start_position

    try:
        _enable(supervisor, args.motor_id)
        enabled = True

        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()

            for amplitude, frequency_hz in args.stages:
                print(f"Running stress stage amplitude={amplitude:.3f} rad freq={frequency_hz:.3f} Hz")
                omega = 2.0 * math.pi * frequency_hz
                duration = args.cycles / frequency_hz
                stage_start = time.monotonic()
                next_report = stage_start
                max_abs_error = 0.0
                max_abs_velocity = 0.0
                max_abs_torque = 0.0
                max_seen_temperature = 0.0
                sample_dt = 1.0 / args.rate_hz

                while True:
                    loop_start = time.monotonic()
                    t = loop_start - stage_start
                    if t > duration:
                        break

                    phase = omega * t
                    target_position = start_position + amplitude * math.sin(phase)
                    target_velocity = amplitude * omega * math.cos(phase)
                    _send_position(
                        supervisor,
                        args.motor_id,
                        target_position,
                        target_velocity,
                        args.kp,
                        args.kd,
                    )

                    state = _require_state(
                        supervisor,
                        args.motor_id,
                        timeout_seconds=args.feedback_timeout,
                        poll_seconds=args.feedback_poll,
                    )
                    previous_unwrapped = _unwrap_position(state.position, previous_unwrapped)
                    tracking_error = previous_unwrapped - target_position
                    max_abs_error = max(max_abs_error, abs(tracking_error))
                    if state.velocity is not None:
                        max_abs_velocity = max(max_abs_velocity, abs(state.velocity))
                    if state.torque is not None:
                        max_abs_torque = max(max_abs_torque, abs(state.torque))
                    if state.temperature is not None:
                        max_seen_temperature = max(max_seen_temperature, state.temperature)

                    if state.temperature is not None and state.temperature > args.max_temperature:
                        raise RuntimeError(f"temperature too high: {state.temperature:.6f} C")
                    if state.velocity is not None and abs(state.velocity) > args.max_velocity:
                        raise RuntimeError(f"velocity too high: {state.velocity:.6f} rad/s")
                    if state.torque is not None and abs(state.torque) > args.max_feedback_torque:
                        raise RuntimeError(f"feedback torque too high: {state.torque:.6f} Nm")
                    if abs(tracking_error) > args.max_tracking_error:
                        raise RuntimeError(
                            f"tracking error too high: {tracking_error:.6f} rad"
                        )
                    if abs(previous_unwrapped - start_position) > amplitude + 0.75:
                        raise RuntimeError(
                            "position moved outside stress window: "
                            f"{previous_unwrapped - start_position:.6f} rad"
                        )

                    writer.writerow(
                        {
                            "time_s": loop_start - run_start,
                            "amplitude_rad": amplitude,
                            "frequency_hz": frequency_hz,
                            "target_position_rad": target_position,
                            "target_velocity_rad_s": target_velocity,
                            "actuator_id": state.actuator_id,
                            "online": state.online,
                            "position_rad": state.position,
                            "position_unwrapped_rad": previous_unwrapped,
                            "velocity_rad_s": state.velocity,
                            "torque_nm": state.torque,
                            "temperature_c": state.temperature,
                            "position_error_rad": tracking_error,
                        }
                    )

                    now = time.monotonic()
                    if now >= next_report:
                        print(
                            f"  target={target_position:+.3f} rad "
                            f"unwrapped={previous_unwrapped:+.3f} rad "
                            f"err={tracking_error:+.3f} rad | {_format_state(state)}"
                        )
                        next_report = now + 0.35

                    sleep_for = sample_dt - (time.monotonic() - loop_start)
                    if sleep_for > 0.0:
                        time.sleep(sleep_for)

                print(
                    f"Summary amplitude={amplitude:.3f} freq={frequency_hz:.3f}: "
                    f"max_error={max_abs_error:.4f} rad "
                    f"max_vel={max_abs_velocity:.4f} rad/s "
                    f"max_torque={max_abs_torque:.4f} Nm "
                    f"max_temp={max_seen_temperature:.1f} C"
                )
                _return_to_start(
                    supervisor,
                    args.motor_id,
                    start_position=start_position,
                    current_position=previous_unwrapped,
                    kp=args.kp,
                    kd=args.kd,
                    seconds=1.0,
                    rate_hz=args.rate_hz,
                )
                previous_unwrapped = start_position
    finally:
        if enabled:
            print("Sending DISABLE")
            try:
                disabled = supervisor.disable_actuator(args.motor_id, False)
                print(f"Disable sent: {disabled}")
            except Exception as exc:  # noqa: BLE001
                print(f"Disable failed: {exc}", file=sys.stderr)
                raise

    print(f"Stress test complete. CSV log: {csv_path}")


if __name__ == "__main__":
    main()
