from enum import Enum


class ControlMode(str, Enum):
    MONITOR = "monitor"
    TRAJECTORY = "trajectory"
    STREAMING = "streaming"


def parse_control_mode(value: str) -> ControlMode:
    normalized = value.strip().lower()
    for mode in ControlMode:
        if mode.value == normalized:
            return mode
    supported = ", ".join(mode.value for mode in ControlMode)
    raise ValueError(f"Unsupported control mode '{value}'. Supported modes: {supported}")


def command_output_active(
    control_mode: ControlMode,
    publish_commands: bool,
    command_output_armed: bool,
) -> bool:
    return (
        control_mode == ControlMode.STREAMING
        and publish_commands
        and command_output_armed
    )
