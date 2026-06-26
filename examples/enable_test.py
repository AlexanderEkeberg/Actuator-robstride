"""Short gated enable/disable test for one RobStride motor.

This script does not send position, velocity, torque, zero, ID-change, or
parameter-write commands. It only requests feedback, sends enable, monitors
feedback briefly, and sends disable in a finally block.
"""

import argparse
import sys
import time
from typing import Any

from actuator import CH341TransportWrapper, RobstrideActuator, RobstrideActuatorConfig


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


def _read_state(supervisor: Any, motor_id: int, settle_seconds: float = 0.08) -> Any | None:
    supervisor.request_feedback(motor_id)
    time.sleep(settle_seconds)
    states = supervisor.get_actuators_state([motor_id])
    return states[0] if states else None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port-name", required=True)
    parser.add_argument("--motor-id", type=int, required=True)
    parser.add_argument("--motor-type", type=int, required=True)
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--dangerous-enable", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--max-pre-velocity", type=float, default=0.5)
    parser.add_argument("--max-enable-velocity", type=float, default=2.0)
    parser.add_argument("--max-position-delta", type=float, default=0.25)
    parser.add_argument("--max-temperature", type=float, default=60.0)
    args = parser.parse_args()

    if not args.dangerous_enable or args.confirm != "ENABLE":
        raise SystemExit(
            "Refusing to enable. Re-run with: --dangerous-enable --confirm ENABLE"
        )

    if args.hold_seconds <= 0.0 or args.hold_seconds > 5.0:
        raise SystemExit("--hold-seconds must be > 0 and <= 5 for this first test")

    transport = CH341TransportWrapper(args.port_name)
    supervisor = RobstrideActuator(
        transports=[transport],
        py_actuators_config=[(args.motor_id, RobstrideActuatorConfig(args.motor_type))],
    )

    discovered = supervisor.get_discovered_ids()
    print(f"Discovered motor IDs: {discovered}")
    if args.motor_id not in discovered:
        raise SystemExit(f"Motor ID {args.motor_id} was not discovered")

    print("Pre-enable feedback:")
    pre_state = None
    for _ in range(5):
        pre_state = _read_state(supervisor, args.motor_id)
        if pre_state is None:
            raise SystemExit("No feedback before enable")
        print(f"  {_format_state(pre_state)}")

    if pre_state.velocity is not None and abs(pre_state.velocity) > args.max_pre_velocity:
        raise SystemExit(
            f"Refusing enable: pre-enable velocity is {pre_state.velocity:.6f} rad/s"
        )

    start_position = pre_state.position
    enabled = False
    try:
        print("Sending ENABLE")
        enabled = supervisor.enable_actuator(args.motor_id, True)
        print(f"Enable sent: {enabled}")
        if not enabled:
            raise SystemExit("Enable command was not sent")

        deadline = time.monotonic() + args.hold_seconds
        while time.monotonic() < deadline:
            state = _read_state(supervisor, args.motor_id)
            if state is None:
                raise RuntimeError("No feedback while enabled")

            print(f"  enabled {_format_state(state)}")

            if state.temperature is not None and state.temperature > args.max_temperature:
                raise RuntimeError(
                    f"Temperature too high: {state.temperature:.6f} C"
                )
            if state.velocity is not None and abs(state.velocity) > args.max_enable_velocity:
                raise RuntimeError(
                    f"Velocity too high while enabled: {state.velocity:.6f} rad/s"
                )
            if (
                start_position is not None
                and state.position is not None
                and abs(state.position - start_position) > args.max_position_delta
            ):
                raise RuntimeError(
                    "Position changed too much while enabled: "
                    f"{state.position - start_position:.6f} rad"
                )

            time.sleep(0.15)
    finally:
        if enabled:
            print("Sending DISABLE")
            try:
                disabled = supervisor.disable_actuator(args.motor_id, False)
                print(f"Disable sent: {disabled}")
            except Exception as exc:  # noqa: BLE001
                print(f"Disable failed: {exc}", file=sys.stderr)
                raise

    print("Enable test complete")


if __name__ == "__main__":
    main()
