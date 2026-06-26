"""Simple tests for the Robstride actuators, without needing to be connected to a real actuator."""

import pytest

from actuator import RobstrideActuator, RobstrideActuatorConfig, StubTransportWrapper


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
