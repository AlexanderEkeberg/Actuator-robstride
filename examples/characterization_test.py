"""Gated position tracking characterization for one RobStride motor.

This is a stronger bench test than examples.motion_test:
it commands several sine sweeps around the current position and logs target
and feedback to CSV. It does not write parameters, set zero, change ID, or send
firmware commands. It always disables the motor in a finally block.
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


def _read_state(supervisor: Any, motor_id: int, settle_seconds: float = 0.02) -> Any | None:
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
) -> bool:
    cmd = RobstrideActuatorCommand(motor_id)
    cmd.position = position
    cmd.velocity = velocity
    cmd.torque = 0.0
    return supervisor.command_actuator_now(cmd, kp, kd, True)


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
        raise RuntimeError(f"Temperature too high: {state.temperature:.6f} C")
    if state.velocity is not None and abs(state.velocity) > max_velocity:
        raise RuntimeError(f"Velocity too high: {state.velocity:.6f} rad/s")
    if state.torque is not None and abs(state.torque) > max_feedback_torque:
        raise RuntimeError(f"Feedback torque too high: {state.torque:.6f} Nm")
    if (
        state.position is not None
        and abs(state.position - start_position) > max_delta_from_start
    ):
        raise RuntimeError(
            "Position moved outside expected characterization window: "
            f"{state.position - start_position:.6f} rad"
        )


def _row(
    *,
    elapsed: float,
    amplitude: float,
    target_position: float,
    target_velocity: float,
    state: Any,
) -> dict[str, float | int | bool | None]:
    return {
        "time_s": elapsed,
        "amplitude_rad": amplitude,
        "target_position_rad": target_position,
        "target_velocity_rad_s": target_velocity,
        "actuator_id": state.actuator_id,
        "online": state.online,
        "position_rad": state.position,
        "velocity_rad_s": state.velocity,
        "torque_nm": state.torque,
        "temperature_c": state.temperature,
        "position_error_rad": (
            None if state.position is None else state.position - target_position
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port-name", required=True)
    parser.add_argument("--motor-id", type=int, required=True)
    parser.add_argument("--motor-type", type=int, required=True)
    parser.add_argument("--amplitudes", type=_parse_amplitudes, default=[0.10, 0.25, 0.50])
    parser.add_argument("--frequency-hz", type=float, default=0.25)
    parser.add_argument("--cycles", type=float, default=2.0)
    parser.add_argument("--kp", type=float, default=40.0)
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument("--rate-hz", type=float, default=25.0)
    parser.add_argument("--csv-file", default="robstride_characterization.csv")
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
        or args.confirm != "CHARACTERIZE"
    ):
        raise SystemExit(
            "Refusing characterization. Re-run with: "
            "--dangerous-enable --dangerous-motion --confirm CHARACTERIZE"
        )

    for amplitude in args.amplitudes:
        if not math.isfinite(amplitude) or amplitude <= 0.0:
            raise SystemExit("all amplitudes must be finite and > 0")
        if amplitude > 0.50:
            raise SystemExit("amplitudes must be <= 0.50 rad for this bench test")
    if args.frequency_hz <= 0.05 or args.frequency_hz > 1.0:
        raise SystemExit("--frequency-hz must be > 0.05 and <= 1.0")
    if args.cycles <= 0.0 or args.cycles > 5.0:
        raise SystemExit("--cycles must be > 0 and <= 5")
    if args.kp < 0.0 or args.kp > 80.0:
        raise SystemExit("--kp must be between 0 and 80 for this bench test")
    if args.kd < 0.0 or args.kd > 8.0:
        raise SystemExit("--kd must be between 0 and 8 for this bench test")
    if args.rate_hz < 5.0 or args.rate_hz > 80.0:
        raise SystemExit("--rate-hz must be between 5 and 80")

    transport = CH341TransportWrapper(args.port_name)
    supervisor = RobstrideActuator(
        transports=[transport],
        py_actuators_config=[(args.motor_id, RobstrideActuatorConfig(args.motor_type))],
    )

    discovered = supervisor.get_discovered_ids()
    print(f"Discovered motor IDs: {discovered}")
    if args.motor_id not in discovered:
        raise SystemExit(f"Motor ID {args.motor_id} was not discovered")

    print("Pre-characterization feedback:")
    state = None
    for _ in range(5):
        state = _read_state(supervisor, args.motor_id)
        if state is None:
            raise SystemExit("No feedback before characterization")
        print(f"  {_format_state(state)}")

    if state.position is None:
        raise SystemExit("No position feedback before characterization")
    if state.velocity is not None and abs(state.velocity) > args.max_pre_velocity:
        raise SystemExit(
            f"Refusing characterization: velocity is {state.velocity:.6f} rad/s"
        )

    start_position = state.position
    max_delta_from_start = max(args.amplitudes) + 0.12
    omega = 2.0 * math.pi * args.frequency_hz
    sample_dt = 1.0 / args.rate_hz
    csv_path = Path(args.csv_file)
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

    print(
        "Characterization plan: "
        f"start={start_position:+.6f} rad amplitudes={args.amplitudes} "
        f"frequency={args.frequency_hz} Hz cycles={args.cycles} "
        f"kp={args.kp} kd={args.kd}"
    )
    print(f"CSV log: {csv_path}")
    print("Keep clear of the output shaft and be ready to cut motor power.")

    enabled = False
    run_start = time.monotonic()

    try:
        print("Sending ENABLE")
        enabled = supervisor.enable_actuator(args.motor_id, True)
        print(f"Enable sent: {enabled}")
        if not enabled:
            raise SystemExit("Enable command was not sent")

        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()

            for amplitude in args.amplitudes:
                print(f"Running sine sweep amplitude={amplitude:.3f} rad")
                sweep_duration = args.cycles / args.frequency_hz
                sweep_start = time.monotonic()
                next_report = sweep_start
                max_abs_error = 0.0
                max_abs_velocity = 0.0
                max_abs_torque = 0.0
                max_temperature = 0.0

                while True:
                    now = time.monotonic()
                    t = now - sweep_start
                    if t > sweep_duration:
                        break

                    phase = omega * t
                    target_position = start_position + amplitude * math.sin(phase)
                    target_velocity = amplitude * omega * math.cos(phase)

                    sent = _send_position(
                        supervisor,
                        args.motor_id,
                        target_position,
                        target_velocity,
                        args.kp,
                        args.kd,
                    )
                    if not sent:
                        raise RuntimeError("Characterization command was not sent")

                    state = _read_state(supervisor, args.motor_id)
                    if state is None:
                        raise RuntimeError("No feedback during characterization")

                    _check_state(
                        state,
                        start_position=start_position,
                        max_delta_from_start=max_delta_from_start,
                        max_velocity=args.max_velocity,
                        max_feedback_torque=args.max_feedback_torque,
                        max_temperature=args.max_temperature,
                    )

                    writer.writerow(
                        _row(
                            elapsed=now - run_start,
                            amplitude=amplitude,
                            target_position=target_position,
                            target_velocity=target_velocity,
                            state=state,
                        )
                    )

                    if state.position is not None:
                        max_abs_error = max(
                            max_abs_error, abs(state.position - target_position)
                        )
                    if state.velocity is not None:
                        max_abs_velocity = max(max_abs_velocity, abs(state.velocity))
                    if state.torque is not None:
                        max_abs_torque = max(max_abs_torque, abs(state.torque))
                    if state.temperature is not None:
                        max_temperature = max(max_temperature, state.temperature)

                    if now >= next_report:
                        print(
                            f"  target={target_position:+.4f} rad "
                            f"err={max_abs_error:.4f} rad | {_format_state(state)}"
                        )
                        next_report = now + 0.5

                    sleep_for = sample_dt - (time.monotonic() - now)
                    if sleep_for > 0.0:
                        time.sleep(sleep_for)

                print(
                    f"Summary amplitude={amplitude:.3f} rad: "
                    f"max_error={max_abs_error:.4f} rad "
                    f"max_vel={max_abs_velocity:.4f} rad/s "
                    f"max_torque={max_abs_torque:.4f} Nm "
                    f"max_temp={max_temperature:.1f} C"
                )

            print("Returning to start")
            for _ in range(max(10, int(args.rate_hz * 0.5))):
                _send_position(
                    supervisor,
                    args.motor_id,
                    start_position,
                    0.0,
                    args.kp,
                    args.kd,
                )
                time.sleep(sample_dt)
    finally:
        if enabled:
            print("Sending DISABLE")
            try:
                disabled = supervisor.disable_actuator(args.motor_id, False)
                print(f"Disable sent: {disabled}")
            except Exception as exc:  # noqa: BLE001
                print(f"Disable failed: {exc}", file=sys.stderr)
                raise

    print("Characterization complete")


if __name__ == "__main__":
    main()
