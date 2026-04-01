from abb_pi0_bridge.bridge_core import PolicyObservation
from abb_pi0_bridge.protocol import build_policy_request, extract_joint_positions


def test_build_policy_request_contains_robot_metadata():
    observation = PolicyObservation(
        stamp_sec=1.0,
        joint_names=("joint_1", "joint_2"),
        positions=(0.1, -0.2),
        velocities=(0.0, 0.0),
        efforts=(0.0, 0.0),
    )

    payload = build_policy_request(observation)

    assert payload["api_version"] == "2026-03-30"
    assert payload["robot"]["vendor"] == "ABB"
    assert payload["robot"]["model"] == "abb_irb6700"
    assert payload["robot"]["action_space"] == "joint_position"
    assert payload["observation"]["joint_positions"] == [0.1, -0.2]


def test_extract_joint_positions_accepts_action_wrapper():
    positions = extract_joint_positions(
        {"action": {"joint_positions": [0.2, -0.3]}},
        ("joint_1", "joint_2"),
    )

    assert positions == (0.2, -0.3)
