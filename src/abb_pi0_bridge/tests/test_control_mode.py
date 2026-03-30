from abb_pi0_bridge.control_mode import ControlMode, command_output_active, parse_control_mode


def test_parse_control_mode_normalizes_case():
    assert parse_control_mode("STREAMING") == ControlMode.STREAMING
    assert parse_control_mode("trajectory") == ControlMode.TRAJECTORY


def test_command_output_active_requires_streaming_mode_and_arm():
    assert command_output_active(ControlMode.STREAMING, True, True) is True
    assert command_output_active(ControlMode.STREAMING, True, False) is False
    assert command_output_active(ControlMode.TRAJECTORY, True, True) is False
