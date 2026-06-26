"""Gated tiny-motion test for one RobStride motor.

The script performs a small relative ramp from the current position and back.
It does not write parameters, set zero, change ID, or send firmware commands.
It always sends disable in a finally block after enable.
"""

import argparse
import math
import sys
import time
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


def _read_state(supervisor: Any, motor_id: int, settle_seconds: float = 0.04) -> Any | None:
    supervisor.request_feedback(motor_id)
    time.sleep(settle_seconds)
    states = supervisor.get_actuators_state([motor_id])
    return states[0] if states else None


def _send_position(
    supervisor: Any,
    motor_id: int,
    position: float,
    kp: float,
    kd: float,
) -> bool:
    cmd = RobstrideActuatorCommand(motor_id)
    cmd.position = position
    cmd.velocity = 0.0
    cmd.torque = 0.0
    return supervisor.command_actuator_now(cmd, kp, kd, True)


def _check_state(
    state: Any,
    *,
    start_position: float,
    allowed_delta: float,
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
    if state.position is not None and abs(state.position - start_position) > allowed_delta:
        raise RuntimeError(
            "Position moved outside expected window: "
            f"{state.position - start_position:.6f} rad"
        )


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
    allowed_delta: float,
    max_velocity: float,
    max_feedback_torque: float,
    max_temperature: float,
) -> None:
    steps = max(2, int(duration * rate_hz))
    for step in range(steps + 1):
        alpha = step / steps
        target = from_position + (to_position - from_position) * alpha
        sent = _send_position(supervisor, motor_id, target, kp, kd)
        if not sent:
            raise RuntimeError("Motion command was not sent")

        state = _read_state(supervisor, motor_id)
        if state is None:
            raise RuntimeError("No feedback during motion")

        print(f"  target={target:+.6f} rad | {_format_state(state)}")
        _check_state(
            state,
            start_position=start_position,
            allowed_delta=allowed_delta,
            max_velocity=max_velocity,
            max_feedback_torque=max_feedback_torque,
            max_temperature=max_temperature,
        )
        time.sleep(1.0 / rate_hz)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port-name", required=True)
    parser.add_argument("--motor-id", type=int, required=True)
    parser.add_argument("--motor-type", type=int, required=True)
    parser.add_argument("--step-rad", type=float, default=0.03)
    parser.add_argument("--kp", type=float, default=15.0)
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument("--ramp-seconds", type=float, default=1.0)
    parser.add_argument("--hold-seconds", type=float, default=0.4)
    parser.add_argument("--rate-hz", type=float, default=15.0)
    parser.add_argument("--max-pre-velocity", type=float, default=0.5)
    parser.add_argument("--max-velocity", type=float, default=2.0)
    parser.add_argument("--max-feedback-torque", type=float, default=2.0)
    parser.add_argument("--max-temperature", type=float, default=60.0)
    parser.add_argument("--dangerous-enable", action="store_true")
    parser.add_argument("--dangerous-motion", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()

    if (
        not args.dangerous_enable
        or not args.dangerous_motion
        or args.confirm != "MOVE"
    ):
        raise SystemExit(
            "Refusing motion. Re-run with: "
            "--dangerous-enable --dangerous-motion --confirm MOVE"
        )

    if not math.isfinite(args.step_rad) or abs(args.step_rad) <= 0.0:
        raise SystemExit("--step-rad must be finite and nonzero")
    if abs(args.step_rad) > 0.10:
        raise SystemExit("--step-rad must be <= 0.10 rad for this bench test")
    if args.kp < 0.0 or args.kp > 50.0:
        raise SystemExit("--kp must be between 0 and 50 for this bench test")
    if args.kd < 0.0 or args.kd > 5.0:
        raise SystemExit("--kd must be between 0 and 5 for this bench test")
    if args.ramp_seconds <= 0.2 or args.ramp_seconds > 5.0:
        raise SystemExit("--ramp-seconds must be > 0.2 and <= 5")
    if args.rate_hz < 2.0 or args.rate_hz > 50.0:
        raise SystemExit("--rate-hz must be between 2 and 50")

    transport = CH341TransportWrapper(args.port_name)
    supervisor = RobstrideActuator(
        transports=[transport],
        py_actuators_config=[(args.motor_id, RobstrideActuatorConfig(args.motor_type))],
    )

    discovered = supervisor.get_discovered_ids()
    print(f"Discovered motor IDs: {discovered}")
    if args.motor_id not in discovered:
        raise SystemExit(f"Motor ID {args.motor_id} was not discovered")

    print("Pre-motion feedback:")
    state = None
    for _ in range(5):
        state = _read_state(supervisor, args.motor_id)
        if state is None:
            raise SystemExit("No feedback before motion")
        print(f"  {_format_state(state)}")

    if state.position is None:
        raise SystemExit("No position feedback before motion")
    if state.velocity is not None and abs(state.velocity) > args.max_pre_velocity:
        raise SystemExit(
            f"Refusing motion: pre-motion velocity is {state.velocity:.6f} rad/s"
        )

    start_position = state.position
    target_position = start_position + args.step_rad
    allowed_delta = abs(args.step_rad) + 0.08
    enabled = False

    print(
        "Tiny motion plan: "
        f"{start_position:+.6f} rad -> {target_position:+.6f} rad -> "
        f"{start_position:+.6f} rad | kp={args.kp} kd={args.kd}"
    )
    print("Keep clear of the output shaft and be ready to cut motor power.")

    try:
        print("Sending ENABLE")
        enabled = supervisor.enable_actuator(args.motor_id, True)
        print(f"Enable sent: {enabled}")
        if not enabled:
            raise SystemExit("Enable command was not sent")

        print("Ramping out")
        _ramp(
            supervisor,
            args.motor_id,
            start_position=start_position,
            from_position=start_position,
            to_position=target_position,
            kp=args.kp,
            kd=args.kd,
            duration=args.ramp_seconds,
            rate_hz=args.rate_hz,
            allowed_delta=allowed_delta,
            max_velocity=args.max_velocity,
            max_feedback_torque=args.max_feedback_torque,
            max_temperature=args.max_temperature,
        )

        hold_until = time.monotonic() + args.hold_seconds
        while time.monotonic() < hold_until:
            _send_position(supervisor, args.motor_id, target_position, args.kp, args.kd)
            state = _read_state(supervisor, args.motor_id)
            if state is None:
                raise RuntimeError("No feedback while holding target")
            print(f"  hold target={target_position:+.6f} rad | {_format_state(state)}")
            _check_state(
                state,
                start_position=start_position,
                allowed_delta=allowed_delta,
                max_velocity=args.max_velocity,
                max_feedback_torque=args.max_feedback_torque,
                max_temperature=args.max_temperature,
            )

        print("Ramping back")
        _ramp(
            supervisor,
            args.motor_id,
            start_position=start_position,
            from_position=target_position,
            to_position=start_position,
            kp=args.kp,
            kd=args.kd,
            duration=args.ramp_seconds,
            rate_hz=args.rate_hz,
            allowed_delta=allowed_delta,
            max_velocity=args.max_velocity,
            max_feedback_torque=args.max_feedback_torque,
            max_temperature=args.max_temperature,
        )
    finally:
        if enabled:
            print("Sending DISABLE")
            try:
                disabled = supervisor.disable_actuator(args.motor_id, False)
                print(f"Disable sent: {disabled}")
            except Exception as exc:  # noqa: BLE001
                print(f"Disable failed: {exc}", file=sys.stderr)
                raise

    print("Tiny motion test complete")


if __name__ == "__main__":
    main()
