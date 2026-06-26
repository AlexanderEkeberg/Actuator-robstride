"""Run the standard gated bench sequence for one RobStride motor.

Sequence:
  1. Discover and read feedback
  2. Enable-only hold
  3. Small ramp out/back
  4. Sine characterization with CSV logging
  5. Optional extended larger ramp test
  6. Optional 360 degree spin test

The script does not write parameters, set zero, change ID, or send firmware
commands. Any stage that enables the motor disables it again in a finally block.
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


def _parse_amplitudes(value: str) -> list[float]:
    amplitudes = [float(part.strip()) for part in value.split(",") if part.strip()]
    if not amplitudes:
        raise argparse.ArgumentTypeError("expected at least one amplitude")
    return amplitudes


def _read_state(supervisor: Any, motor_id: int, settle_seconds: float = 0.03) -> Any | None:
    supervisor.request_feedback(motor_id)
    time.sleep(settle_seconds)
    states = supervisor.get_actuators_state([motor_id])
    return states[0] if states else None


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


def _check_state(
    state: Any,
    *,
    start_position: float,
    max_delta_from_start: float,
    max_velocity: float,
    max_feedback_torque: float,
    max_temperature: float,
) -> None:
    if state.temperature is not None and state.temperature > max_temperature:
        raise RuntimeError(f"temperature too high: {state.temperature:.6f} C")
    if state.velocity is not None and abs(state.velocity) > max_velocity:
        raise RuntimeError(f"velocity too high: {state.velocity:.6f} rad/s")
    if state.torque is not None and abs(state.torque) > max_feedback_torque:
        raise RuntimeError(f"feedback torque too high: {state.torque:.6f} Nm")
    if (
        state.position is not None
        and abs(state.position - start_position) > max_delta_from_start
    ):
        raise RuntimeError(
            "position moved outside expected window: "
            f"{state.position - start_position:.6f} rad"
        )


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _unwrap_position(wrapped_position: float, previous_unwrapped: float) -> float:
    delta = wrapped_position - _wrap_pi(previous_unwrapped)
    if delta > math.pi:
        delta -= 2.0 * math.pi
    elif delta < -math.pi:
        delta += 2.0 * math.pi
    return previous_unwrapped + delta


def _require_state(supervisor: Any, motor_id: int) -> Any:
    state = _read_state(supervisor, motor_id)
    if state is None or state.position is None:
        raise RuntimeError("missing motor feedback")
    return state


def _enable(supervisor: Any, motor_id: int) -> None:
    print("Sending ENABLE")
    if not supervisor.enable_actuator(motor_id, True):
        raise RuntimeError("enable command was not sent")


def _disable(supervisor: Any, motor_id: int) -> None:
    print("Sending DISABLE")
    disabled = supervisor.disable_actuator(motor_id, False)
    print(f"Disable sent: {disabled}")


def _read_stage(
    supervisor: Any,
    motor_id: int,
    *,
    seconds: float,
    max_pre_velocity: float,
) -> Any:
    print("\n[1/4] Read-only feedback")
    deadline = time.monotonic() + seconds
    last_state = None
    while time.monotonic() < deadline:
        last_state = _require_state(supervisor, motor_id)
        print(f"  {_format_state(last_state)}")
        if (
            last_state.velocity is not None
            and abs(last_state.velocity) > max_pre_velocity
        ):
            raise RuntimeError(
                f"motor is moving before test: {last_state.velocity:.6f} rad/s"
            )
        time.sleep(0.25)
    return last_state


def _enable_stage(
    supervisor: Any,
    motor_id: int,
    *,
    start_position: float,
    seconds: float,
    max_position_delta: float,
    max_velocity: float,
    max_feedback_torque: float,
    max_temperature: float,
) -> None:
    print("\n[2/4] Enable-only hold")
    enabled = False
    try:
        _enable(supervisor, motor_id)
        enabled = True
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            state = _require_state(supervisor, motor_id)
            print(f"  enabled {_format_state(state)}")
            _check_state(
                state,
                start_position=start_position,
                max_delta_from_start=max_position_delta,
                max_velocity=max_velocity,
                max_feedback_torque=max_feedback_torque,
                max_temperature=max_temperature,
            )
            time.sleep(0.15)
    finally:
        if enabled:
            _disable(supervisor, motor_id)


def _ramp(
    supervisor: Any,
    motor_id: int,
    *,
    start_position: float,
    from_position: float,
    to_position: float,
    kp: float,
    kd: float,
    duration: float,
    rate_hz: float,
    max_delta_from_start: float,
    max_velocity: float,
    max_feedback_torque: float,
    max_temperature: float,
) -> None:
    steps = max(2, int(duration * rate_hz))
    for step in range(steps + 1):
        alpha = step / steps
        target = from_position + (to_position - from_position) * alpha
        _send_position(supervisor, motor_id, target, 0.0, kp, kd)
        state = _require_state(supervisor, motor_id)
        print(f"  target={target:+.6f} rad | {_format_state(state)}")
        _check_state(
            state,
            start_position=start_position,
            max_delta_from_start=max_delta_from_start,
            max_velocity=max_velocity,
            max_feedback_torque=max_feedback_torque,
            max_temperature=max_temperature,
        )
        time.sleep(1.0 / rate_hz)


def _motion_stage(
    supervisor: Any,
    motor_id: int,
    *,
    start_position: float,
    step_rad: float,
    kp: float,
    kd: float,
    ramp_seconds: float,
    rate_hz: float,
    max_velocity: float,
    max_feedback_torque: float,
    max_temperature: float,
) -> None:
    print("\n[3/4] Ramp motion out/back")
    enabled = False
    try:
        _enable(supervisor, motor_id)
        enabled = True
        target_position = start_position + step_rad
        max_delta_from_start = abs(step_rad) + 0.10
        _ramp(
            supervisor,
            motor_id,
            start_position=start_position,
            from_position=start_position,
            to_position=target_position,
            kp=kp,
            kd=kd,
            duration=ramp_seconds,
            rate_hz=rate_hz,
            max_delta_from_start=max_delta_from_start,
            max_velocity=max_velocity,
            max_feedback_torque=max_feedback_torque,
            max_temperature=max_temperature,
        )
        _ramp(
            supervisor,
            motor_id,
            start_position=start_position,
            from_position=target_position,
            to_position=start_position,
            kp=kp,
            kd=kd,
            duration=ramp_seconds,
            rate_hz=rate_hz,
            max_delta_from_start=max_delta_from_start,
            max_velocity=max_velocity,
            max_feedback_torque=max_feedback_torque,
            max_temperature=max_temperature,
        )
    finally:
        if enabled:
            _disable(supervisor, motor_id)


def _large_motion_stage(
    supervisor: Any,
    motor_id: int,
    *,
    start_position: float,
    step_rad: float,
    kp: float,
    kd: float,
    ramp_seconds: float,
    rate_hz: float,
    max_velocity: float,
    max_feedback_torque: float,
    max_temperature: float,
) -> None:
    print("\n[5/5] Extended larger ramp motion")
    enabled = False
    try:
        _enable(supervisor, motor_id)
        enabled = True
        max_delta_from_start = abs(step_rad) + 0.15
        waypoints = [
            start_position + step_rad,
            start_position,
            start_position - step_rad,
            start_position,
        ]
        current = start_position
        for waypoint in waypoints:
            print(f"  ramp {current:+.6f} rad -> {waypoint:+.6f} rad")
            _ramp(
                supervisor,
                motor_id,
                start_position=start_position,
                from_position=current,
                to_position=waypoint,
                kp=kp,
                kd=kd,
                duration=ramp_seconds,
                rate_hz=rate_hz,
                max_delta_from_start=max_delta_from_start,
                max_velocity=max_velocity,
                max_feedback_torque=max_feedback_torque,
                max_temperature=max_temperature,
            )
            current = waypoint
    finally:
        if enabled:
            _disable(supervisor, motor_id)


def _spin_stage(
    supervisor: Any,
    motor_id: int,
    *,
    start_position: float,
    revolutions: float,
    seconds: float,
    kp: float,
    kd: float,
    rate_hz: float,
    max_velocity: float,
    max_feedback_torque: float,
    max_temperature: float,
    max_tracking_error: float,
) -> None:
    print("\n[6/6] 360 degree spin test")
    enabled = False
    distance = revolutions * 2.0 * math.pi
    steps = max(4, int(seconds * rate_hz))
    sample_dt = 1.0 / rate_hz
    previous_unwrapped = start_position
    max_abs_error = 0.0
    max_abs_velocity = 0.0
    max_abs_torque = 0.0
    max_seen_temperature = 0.0
    next_report = time.monotonic()

    try:
        _enable(supervisor, motor_id)
        enabled = True
        for step in range(steps + 1):
            loop_start = time.monotonic()
            u = step / steps
            smooth = 3.0 * u * u - 2.0 * u * u * u
            smooth_velocity = (6.0 * u - 6.0 * u * u) / seconds
            target_position = start_position + distance * smooth
            target_velocity = distance * smooth_velocity

            _send_position(
                supervisor,
                motor_id,
                target_position,
                target_velocity,
                kp,
                kd,
            )
            state = _require_state(supervisor, motor_id)
            if state.position is None:
                raise RuntimeError("missing position feedback during spin")

            previous_unwrapped = _unwrap_position(state.position, previous_unwrapped)
            tracking_error = previous_unwrapped - target_position
            max_abs_error = max(max_abs_error, abs(tracking_error))
            if state.velocity is not None:
                max_abs_velocity = max(max_abs_velocity, abs(state.velocity))
            if state.torque is not None:
                max_abs_torque = max(max_abs_torque, abs(state.torque))
            if state.temperature is not None:
                max_seen_temperature = max(max_seen_temperature, state.temperature)

            if state.temperature is not None and state.temperature > max_temperature:
                raise RuntimeError(f"temperature too high: {state.temperature:.6f} C")
            if state.velocity is not None and abs(state.velocity) > max_velocity:
                raise RuntimeError(f"velocity too high: {state.velocity:.6f} rad/s")
            if state.torque is not None and abs(state.torque) > max_feedback_torque:
                raise RuntimeError(f"feedback torque too high: {state.torque:.6f} Nm")
            if abs(tracking_error) > max_tracking_error:
                raise RuntimeError(
                    f"spin tracking error too high: {tracking_error:.6f} rad"
                )
            if abs(previous_unwrapped - start_position) > abs(distance) + 0.50:
                raise RuntimeError(
                    "spin moved outside expected window: "
                    f"{previous_unwrapped - start_position:.6f} rad"
                )

            now = time.monotonic()
            if now >= next_report:
                print(
                    f"  target={target_position:+.4f} rad "
                    f"unwrapped={previous_unwrapped:+.4f} rad "
                    f"err={tracking_error:+.4f} rad | {_format_state(state)}"
                )
                next_report = now + 0.35

            sleep_for = sample_dt - (time.monotonic() - loop_start)
            if sleep_for > 0.0:
                time.sleep(sleep_for)

        print(
            "  spin summary: "
            f"max_error={max_abs_error:.4f} rad "
            f"max_vel={max_abs_velocity:.4f} rad/s "
            f"max_torque={max_abs_torque:.4f} Nm "
            f"max_temp={max_seen_temperature:.1f} C"
        )
    finally:
        if enabled:
            _disable(supervisor, motor_id)


def _characterization_stage(
    supervisor: Any,
    motor_id: int,
    *,
    start_position: float,
    amplitudes: list[float],
    frequency_hz: float,
    cycles: float,
    kp: float,
    kd: float,
    rate_hz: float,
    csv_path: Path,
    max_velocity: float,
    max_feedback_torque: float,
    max_temperature: float,
) -> None:
    print("\n[4/4] Sine characterization")
    enabled = False
    omega = 2.0 * math.pi * frequency_hz
    sample_dt = 1.0 / rate_hz
    max_delta_from_start = max(amplitudes) + 0.12
    fieldnames = [
        "time_s",
        "amplitude_rad",
        "target_position_rad",
        "target_velocity_rad_s",
        "actuator_id",
        "online",
        "position_rad",
        "velocity_rad_s",
        "torque_nm",
        "temperature_c",
        "position_error_rad",
    ]
    run_start = time.monotonic()

    try:
        _enable(supervisor, motor_id)
        enabled = True
        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()

            for amplitude in amplitudes:
                print(f"  sine amplitude={amplitude:.3f} rad")
                sweep_duration = cycles / frequency_hz
                sweep_start = time.monotonic()
                next_report = sweep_start
                max_abs_error = 0.0
                max_abs_velocity = 0.0
                max_abs_torque = 0.0
                max_seen_temperature = 0.0

                while True:
                    now = time.monotonic()
                    t = now - sweep_start
                    if t > sweep_duration:
                        break

                    phase = omega * t
                    target_position = start_position + amplitude * math.sin(phase)
                    target_velocity = amplitude * omega * math.cos(phase)
                    _send_position(
                        supervisor,
                        motor_id,
                        target_position,
                        target_velocity,
                        kp,
                        kd,
                    )
                    state = _require_state(supervisor, motor_id)
                    _check_state(
                        state,
                        start_position=start_position,
                        max_delta_from_start=max_delta_from_start,
                        max_velocity=max_velocity,
                        max_feedback_torque=max_feedback_torque,
                        max_temperature=max_temperature,
                    )

                    position_error = (
                        None
                        if state.position is None
                        else state.position - target_position
                    )
                    writer.writerow(
                        {
                            "time_s": now - run_start,
                            "amplitude_rad": amplitude,
                            "target_position_rad": target_position,
                            "target_velocity_rad_s": target_velocity,
                            "actuator_id": state.actuator_id,
                            "online": state.online,
                            "position_rad": state.position,
                            "velocity_rad_s": state.velocity,
                            "torque_nm": state.torque,
                            "temperature_c": state.temperature,
                            "position_error_rad": position_error,
                        }
                    )

                    if position_error is not None:
                        max_abs_error = max(max_abs_error, abs(position_error))
                    if state.velocity is not None:
                        max_abs_velocity = max(max_abs_velocity, abs(state.velocity))
                    if state.torque is not None:
                        max_abs_torque = max(max_abs_torque, abs(state.torque))
                    if state.temperature is not None:
                        max_seen_temperature = max(max_seen_temperature, state.temperature)

                    if now >= next_report:
                        print(
                            f"    target={target_position:+.4f} rad "
                            f"max_err={max_abs_error:.4f} rad | {_format_state(state)}"
                        )
                        next_report = now + 0.5

                    sleep_for = sample_dt - (time.monotonic() - now)
                    if sleep_for > 0.0:
                        time.sleep(sleep_for)

                print(
                    f"  summary amplitude={amplitude:.3f}: "
                    f"max_error={max_abs_error:.4f} rad "
                    f"max_vel={max_abs_velocity:.4f} rad/s "
                    f"max_torque={max_abs_torque:.4f} Nm "
                    f"max_temp={max_seen_temperature:.1f} C"
                )

            print("  returning to start")
            for _ in range(max(10, int(rate_hz * 0.5))):
                _send_position(supervisor, motor_id, start_position, 0.0, kp, kd)
                time.sleep(sample_dt)
    finally:
        if enabled:
            _disable(supervisor, motor_id)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port-name", required=True)
    parser.add_argument("--motor-id", type=int, required=True)
    parser.add_argument("--motor-type", type=int, required=True)
    parser.add_argument(
        "--profile",
        choices=["standard", "extended", "spin"],
        default="standard",
    )
    parser.add_argument("--motion-step-rad", type=float, default=0.20)
    parser.add_argument("--large-step-rad", type=float, default=1.0)
    parser.add_argument("--spin-revolutions", type=float, default=1.0)
    parser.add_argument("--spin-seconds", type=float, default=4.0)
    parser.add_argument("--max-spin-error", type=float, default=1.25)
    parser.add_argument("--amplitudes", type=_parse_amplitudes, default=[0.10, 0.25, 0.50])
    parser.add_argument("--kp", type=float, default=40.0)
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument("--read-seconds", type=float, default=3.0)
    parser.add_argument("--enable-seconds", type=float, default=2.0)
    parser.add_argument("--ramp-seconds", type=float, default=1.0)
    parser.add_argument("--large-ramp-seconds", type=float, default=2.0)
    parser.add_argument("--frequency-hz", type=float, default=0.25)
    parser.add_argument("--cycles", type=float, default=1.0)
    parser.add_argument("--rate-hz", type=float, default=25.0)
    parser.add_argument("--csv-file", default="")
    parser.add_argument("--max-pre-velocity", type=float, default=0.5)
    parser.add_argument("--max-velocity", type=float, default=4.0)
    parser.add_argument("--max-feedback-torque", type=float, default=4.0)
    parser.add_argument("--max-temperature", type=float, default=60.0)
    parser.add_argument("--dangerous-enable", action="store_true")
    parser.add_argument("--dangerous-motion", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()

    if (
        not args.dangerous_enable
        or not args.dangerous_motion
        or args.confirm != "BENCH"
    ):
        raise SystemExit(
            "Refusing bench sequence. Re-run with: "
            "--dangerous-enable --dangerous-motion --confirm BENCH"
        )

    if not math.isfinite(args.motion_step_rad) or abs(args.motion_step_rad) <= 0.0:
        raise SystemExit("--motion-step-rad must be finite and nonzero")
    if abs(args.motion_step_rad) > 0.50:
        raise SystemExit("--motion-step-rad must be <= 0.50 rad")
    if not math.isfinite(args.large_step_rad) or abs(args.large_step_rad) <= 0.0:
        raise SystemExit("--large-step-rad must be finite and nonzero")
    if abs(args.large_step_rad) > 1.0:
        raise SystemExit("--large-step-rad must be <= 1.0 rad")
    if not math.isfinite(args.spin_revolutions) or args.spin_revolutions <= 0.0:
        raise SystemExit("--spin-revolutions must be finite and > 0")
    if args.spin_revolutions > 1.0:
        raise SystemExit("--spin-revolutions must be <= 1.0")
    if args.spin_seconds < 2.5 or args.spin_seconds > 12.0:
        raise SystemExit("--spin-seconds must be between 2.5 and 12")
    if args.max_spin_error <= 0.0 or args.max_spin_error > 2.0:
        raise SystemExit("--max-spin-error must be > 0 and <= 2.0 rad")
    for amplitude in args.amplitudes:
        if not math.isfinite(amplitude) or amplitude <= 0.0:
            raise SystemExit("all amplitudes must be finite and > 0")
        if amplitude > 0.50:
            raise SystemExit("amplitudes must be <= 0.50 rad")
    if args.kp < 0.0 or args.kp > 80.0:
        raise SystemExit("--kp must be between 0 and 80")
    if args.kd < 0.0 or args.kd > 8.0:
        raise SystemExit("--kd must be between 0 and 8")
    if args.frequency_hz <= 0.05 or args.frequency_hz > 1.0:
        raise SystemExit("--frequency-hz must be > 0.05 and <= 1.0")
    if args.cycles <= 0.0 or args.cycles > 5.0:
        raise SystemExit("--cycles must be > 0 and <= 5")
    if args.large_ramp_seconds <= 0.5 or args.large_ramp_seconds > 8.0:
        raise SystemExit("--large-ramp-seconds must be > 0.5 and <= 8")
    if args.rate_hz < 5.0 or args.rate_hz > 80.0:
        raise SystemExit("--rate-hz must be between 5 and 80")

    csv_path = (
        Path(args.csv_file)
        if args.csv_file
        else Path(
            f"robstride_bench_type{args.motor_type}_id{args.motor_id}_"
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

    print(
        "Bench sequence: "
        f"profile={args.profile} type={args.motor_type} id={args.motor_id} "
        f"step={args.motion_step_rad} rad amplitudes={args.amplitudes} "
        f"kp={args.kp} kd={args.kd}"
    )
    print("Keep clear of the output shaft and be ready to cut motor power.")

    start_state = _read_stage(
        supervisor,
        args.motor_id,
        seconds=args.read_seconds,
        max_pre_velocity=args.max_pre_velocity,
    )
    start_position = start_state.position
    if start_position is None:
        raise SystemExit("No start position feedback")

    _enable_stage(
        supervisor,
        args.motor_id,
        start_position=start_position,
        seconds=args.enable_seconds,
        max_position_delta=0.25,
        max_velocity=args.max_velocity,
        max_feedback_torque=args.max_feedback_torque,
        max_temperature=args.max_temperature,
    )
    _motion_stage(
        supervisor,
        args.motor_id,
        start_position=start_position,
        step_rad=args.motion_step_rad,
        kp=args.kp,
        kd=args.kd,
        ramp_seconds=args.ramp_seconds,
        rate_hz=args.rate_hz,
        max_velocity=args.max_velocity,
        max_feedback_torque=args.max_feedback_torque,
        max_temperature=args.max_temperature,
    )
    _characterization_stage(
        supervisor,
        args.motor_id,
        start_position=start_position,
        amplitudes=args.amplitudes,
        frequency_hz=args.frequency_hz,
        cycles=args.cycles,
        kp=args.kp,
        kd=args.kd,
        rate_hz=args.rate_hz,
        csv_path=csv_path,
        max_velocity=args.max_velocity,
        max_feedback_torque=args.max_feedback_torque,
        max_temperature=args.max_temperature,
    )
    if args.profile in {"extended", "spin"}:
        _large_motion_stage(
            supervisor,
            args.motor_id,
            start_position=start_position,
            step_rad=args.large_step_rad,
            kp=args.kp,
            kd=args.kd,
            ramp_seconds=args.large_ramp_seconds,
            rate_hz=args.rate_hz,
            max_velocity=args.max_velocity,
            max_feedback_torque=args.max_feedback_torque,
            max_temperature=args.max_temperature,
        )
    if args.profile == "spin":
        _spin_stage(
            supervisor,
            args.motor_id,
            start_position=start_position,
            revolutions=args.spin_revolutions,
            seconds=args.spin_seconds,
            kp=args.kp,
            kd=args.kd,
            rate_hz=args.rate_hz,
            max_velocity=args.max_velocity,
            max_feedback_torque=args.max_feedback_torque,
            max_temperature=args.max_temperature,
            max_tracking_error=args.max_spin_error,
        )

    print(f"\nBench sequence complete. CSV log: {csv_path}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted", file=sys.stderr)
        raise
