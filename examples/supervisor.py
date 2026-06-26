"""Example of reading motor state using the supervisor."""

import argparse
import time

from actuator import CH341TransportWrapper, RobstrideActuator, RobstrideActuatorConfig


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port-name", type=str, default="/dev/ttyCH341USB0")
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--motor-type", type=int, default=4)
    parser.add_argument("--sleep", type=float, default=0.0)
    parser.add_argument("--period", type=float, default=10.0)
    parser.add_argument("--amplitude", type=float, default=1.0)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    _amplitude = args.amplitude
    _period = args.period

    transport = CH341TransportWrapper(args.port_name)

    supervisor = RobstrideActuator(
        transports=[transport],
        py_actuators_config=[(args.motor_id, RobstrideActuatorConfig(args.motor_type))],
    )

    print(f"Discovered motor IDs: {supervisor.get_discovered_ids()}")

    while True:
        requested = supervisor.request_feedback(args.motor_id)
        time.sleep(0.05)
        print(
            f"Feedback requested: {requested} | "
            f"State (rad, rad/s, Nm): {supervisor.get_actuators_state([args.motor_id])}"
        )
        time.sleep(0.95)


if __name__ == "__main__":
    # python -m examples.supervisor
    main()
