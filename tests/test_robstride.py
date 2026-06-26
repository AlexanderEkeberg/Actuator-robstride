"""Simple tests for the Robstride actuators, without needing to be connected to a real actuator."""

import pytest

from actuator import (
    RobstrideActuator,
    RobstrideActuatorCommand,
    RobstrideActuatorConfig,
    StubTransportWrapper,
)


def test_robstride() -> None:
    # Create transport object instead of using port string
    stub_transport = StubTransportWrapper("stub")

    supervisor = RobstrideActuator(
        transports=[stub_transport],
        py_actuators_config=[(1, RobstrideActuatorConfig(1))],
    )

    for _ in range(3):
        state = supervisor.get_actuators_state([1])
        assert isinstance(state, list)  # State is empty for now.

    assert supervisor.get_discovered_ids() == []
    assert supervisor.request_feedback(1) is False


def test_robstride06_config_is_accepted() -> None:
    stub_transport = StubTransportWrapper("stub")

    supervisor = RobstrideActuator(
        transports=[stub_transport],
        py_actuators_config=[(1, RobstrideActuatorConfig(6))],
    )

    assert supervisor.get_actuators_state([1]) == []
    assert supervisor.get_discovered_ids() == []
    assert supervisor.request_feedback(1) is False


def test_unknown_actuator_type_raises_error() -> None:
    stub_transport = StubTransportWrapper("stub")

    with pytest.raises(ValueError, match="Unknown actuator type: 5"):
        RobstrideActuator(
            transports=[stub_transport],
            py_actuators_config=[(1, RobstrideActuatorConfig(5))],
        )


def test_enable_actuator_requires_dangerous_flag() -> None:
    stub_transport = StubTransportWrapper("stub")

    supervisor = RobstrideActuator(
        transports=[stub_transport],
        py_actuators_config=[(1, RobstrideActuatorConfig(6))],
    )

    with pytest.raises(ValueError, match="dangerous_enable=True"):
        supervisor.enable_actuator(1, False)

    assert supervisor.enable_actuator(1, True) is False
    assert supervisor.disable_actuator(1, False) is False


def test_command_actuator_now_requires_motion_gate_and_position() -> None:
    stub_transport = StubTransportWrapper("stub")

    supervisor = RobstrideActuator(
        transports=[stub_transport],
        py_actuators_config=[(1, RobstrideActuatorConfig(6))],
    )

    cmd = RobstrideActuatorCommand(1)
    cmd.position = 0.0

    with pytest.raises(ValueError, match="dangerous_motion=True"):
        supervisor.command_actuator_now(cmd, 10.0, 1.0, False)

    missing_position = RobstrideActuatorCommand(1)
    with pytest.raises(ValueError, match="cmd.position"):
        supervisor.command_actuator_now(missing_position, 10.0, 1.0, True)

    assert supervisor.command_actuator_now(cmd, 10.0, 1.0, True) is False
