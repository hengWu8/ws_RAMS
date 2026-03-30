from abb_pi0_bridge.bridge_core import (
    PolicyObservation,
    SafetyConfig,
    apply_safety_filters,
    build_observation,
    observation_is_fresh,
)
from abb_pi0_bridge.mock_policy import MockHoldPolicy


def test_build_observation_orders_joint_state_vectors():
    observation = build_observation(
        ordered_joint_names=["joint_1", "joint_2", "joint_3"],
        msg_joint_names=["joint_3", "joint_1", "joint_2"],
        positions=[3.0, 1.0, 2.0],
        velocities=[0.3, 0.1, 0.2],
        efforts=[],
        stamp_sec=12.5,
    )

    assert observation == PolicyObservation(
        stamp_sec=12.5,
        joint_names=("joint_1", "joint_2", "joint_3"),
        positions=(1.0, 2.0, 3.0),
        velocities=(0.1, 0.2, 0.3),
        efforts=(0.0, 0.0, 0.0),
    )


def test_apply_safety_filters_clamps_limits_and_step_size():
    safe_command = apply_safety_filters(
        current_positions=[0.0, 0.0],
        requested_positions=[1.0, -2.0],
        safety_config=SafetyConfig(
            lower_limits=(-0.5, -0.5),
            upper_limits=(0.5, 0.5),
            max_position_step_rad=0.1,
        ),
    )

    assert safe_command.positions == (0.1, -0.1)
    assert safe_command.was_clamped is True
    assert "joint_0_limit" in safe_command.reason
    assert "joint_1_limit" in safe_command.reason


def test_observation_freshness_check():
    assert observation_is_fresh(now_sec=10.1, observation_stamp_sec=10.0, timeout_sec=0.5) is True
    assert observation_is_fresh(now_sec=10.8, observation_stamp_sec=10.0, timeout_sec=0.5) is False


def test_mock_policy_returns_current_joint_positions():
    policy = MockHoldPolicy()
    observation = PolicyObservation(
        stamp_sec=1.0,
        joint_names=("joint_1", "joint_2"),
        positions=(0.2, -0.4),
        velocities=(0.0, 0.0),
        efforts=(0.0, 0.0),
    )

    assert policy.compute_command(observation) == (0.2, -0.4)
