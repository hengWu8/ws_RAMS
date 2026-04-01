from typing import Any

from .bridge_core import PolicyObservation


def build_policy_request(
    observation: PolicyObservation,
    control_mode: str = "streaming",
    robot_model: str = "abb_irb6700",
    action_space: str = "joint_position",
) -> dict[str, Any]:
    return {
        "api_version": "2026-03-30",
        "control_mode": control_mode,
        "robot": {
            "vendor": "ABB",
            "model": robot_model,
            "joint_names": list(observation.joint_names),
            "action_space": action_space,
        },
        "observation": observation.as_policy_input(),
    }


def extract_joint_positions(
    response_payload: dict[str, Any],
    expected_joint_names: tuple[str, ...],
) -> tuple[float, ...]:
    joint_positions = response_payload.get("joint_positions")
    if joint_positions is None and isinstance(response_payload.get("action"), dict):
        joint_positions = response_payload["action"].get("joint_positions")
    if joint_positions is None:
        joint_positions = response_payload.get("target_joint_positions")

    if isinstance(joint_positions, dict):
        missing = [name for name in expected_joint_names if name not in joint_positions]
        if missing:
            raise ValueError(f"Policy response is missing joint positions for: {missing}")
        return tuple(float(joint_positions[name]) for name in expected_joint_names)

    if isinstance(joint_positions, list):
        if len(joint_positions) != len(expected_joint_names):
            raise ValueError(
                "Policy response joint_positions length does not match the expected ABB joint vector."
            )
        return tuple(float(value) for value in joint_positions)

    raise ValueError("Policy response does not contain a supported joint_positions field.")
