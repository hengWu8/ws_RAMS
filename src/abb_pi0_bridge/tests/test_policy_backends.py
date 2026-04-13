import json

import pytest

from abb_pi0_bridge.bridge_core import PolicyObservation
from abb_pi0_bridge.cartesian_workspace import UrdfForwardKinematics
from abb_pi0_bridge.mock_policy import (
    CartesianRampPolicy,
    CartesianStepPolicy,
    CartesianTimedTrajectoryPolicy,
    CartesianTrueFeedbackServoPolicy,
    HttpPolicyServerClient,
    extract_joint_positions,
)


SIMPLE_URDF = """
<robot name="simple">
  <link name="base_link"/>
  <link name="link_1"/>
  <link name="tcp"/>
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="tool" type="fixed">
    <parent link="link_1"/>
    <child link="tcp"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


class _FakeResponse:
    def __init__(self, payload: dict):
        self._payload = payload

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return False

    def read(self):
        return json.dumps(self._payload).encode("utf-8")


def test_extract_joint_positions_accepts_joint_dict():
    positions = extract_joint_positions(
        {"joint_positions": {"joint_1": 0.1, "joint_2": -0.2}},
        ("joint_1", "joint_2"),
    )

    assert positions == (0.1, -0.2)


def test_http_policy_server_client_parses_response():
    observation = PolicyObservation(
        stamp_sec=1.0,
        joint_names=("joint_1", "joint_2"),
        positions=(0.0, 0.0),
        velocities=(0.0, 0.0),
        efforts=(0.0, 0.0),
    )

    client = HttpPolicyServerClient(
        server_url="http://localhost:8000/infer",
        timeout_sec=0.2,
        urlopen=lambda request, timeout: _FakeResponse({"joint_positions": [0.3, -0.4]}),
    )

    assert client.compute_command(observation) == (0.3, -0.4)


def test_extract_joint_positions_rejects_missing_field():
    with pytest.raises(ValueError):
        extract_joint_positions({}, ("joint_1", "joint_2"))


def test_cartesian_step_policy_moves_tip_toward_base_y():
    fk = UrdfForwardKinematics.from_robot_description(
        SIMPLE_URDF,
        base_link="base_link",
        tip_link="tcp",
    )
    policy = CartesianStepPolicy(
        forward_kinematics=fk,
        direction_xyz=(0.0, 1.0, 0.0),
        step_m=0.001,
        damping=0.01,
        max_joint_delta_rad=0.1,
    )
    observation = PolicyObservation(
        stamp_sec=1.0,
        joint_names=("joint_1",),
        positions=(0.0,),
        velocities=(0.0,),
        efforts=(0.0,),
    )

    commanded = policy.compute_command(observation)
    current_tip = fk.compute_tip_position(observation.joint_names, observation.positions)
    commanded_tip = fk.compute_tip_position(observation.joint_names, commanded)

    assert commanded[0] > 0.0
    assert commanded_tip[1] > current_tip[1]


def test_cartesian_ramp_policy_accumulates_target_motion():
    fk = UrdfForwardKinematics.from_robot_description(
        SIMPLE_URDF,
        base_link="base_link",
        tip_link="tcp",
    )
    policy = CartesianRampPolicy(
        forward_kinematics=fk,
        direction_xyz=(0.0, 1.0, 0.0),
        step_m=0.001,
        damping=0.01,
        max_joint_delta_rad=0.1,
    )
    observation = PolicyObservation(
        stamp_sec=1.0,
        joint_names=("joint_1",),
        positions=(0.0,),
        velocities=(0.0,),
        efforts=(0.0,),
    )

    first_command = policy.compute_command(observation)
    second_command = policy.compute_command(observation)

    first_tip = fk.compute_tip_position(observation.joint_names, first_command)
    second_tip = fk.compute_tip_position(observation.joint_names, second_command)

    assert second_command[0] > first_command[0]
    assert second_tip[1] > first_tip[1]


def test_cartesian_timed_trajectory_policy_advances_with_elapsed_time():
    fk = UrdfForwardKinematics.from_robot_description(
        SIMPLE_URDF,
        base_link="base_link",
        tip_link="tcp",
    )
    policy = CartesianTimedTrajectoryPolicy(
        forward_kinematics=fk,
        direction_xyz=(0.0, 1.0, 0.0),
        speed_mps=0.001,
        damping=0.01,
        max_joint_delta_rad=0.1,
    )
    obs_t0 = PolicyObservation(
        stamp_sec=1.0,
        joint_names=("joint_1",),
        positions=(0.0,),
        velocities=(0.0,),
        efforts=(0.0,),
    )
    obs_t1 = PolicyObservation(
        stamp_sec=2.0,
        joint_names=("joint_1",),
        positions=(0.0,),
        velocities=(0.0,),
        efforts=(0.0,),
    )

    first_command = policy.compute_command(obs_t0)
    second_command = policy.compute_command(obs_t1)

    first_tip = fk.compute_tip_position(obs_t0.joint_names, first_command)
    second_tip = fk.compute_tip_position(obs_t1.joint_names, second_command)

    assert second_command[0] > first_command[0]
    assert second_tip[1] > first_tip[1]


def test_cartesian_true_feedback_servo_policy_uses_feedback_error():
    fk = UrdfForwardKinematics.from_robot_description(
        SIMPLE_URDF,
        base_link="base_link",
        tip_link="tcp",
    )
    feedback_state = {
        "observation": PolicyObservation(
            stamp_sec=1.0,
            joint_names=("joint_1",),
            positions=(0.0,),
            velocities=(0.0,),
            efforts=(0.0,),
        )
    }
    policy = CartesianTrueFeedbackServoPolicy(
        forward_kinematics=fk,
        feedback_observation_provider=lambda: feedback_state["observation"],
        direction_xyz=(0.0, 1.0, 0.0),
        speed_mps=0.001,
        tracking_gain=1.0,
        max_cartesian_error_m=0.02,
        damping=0.01,
        max_joint_delta_rad=0.1,
    )
    obs_t0 = PolicyObservation(
        stamp_sec=1.0,
        joint_names=("joint_1",),
        positions=(0.0,),
        velocities=(0.0,),
        efforts=(0.0,),
    )
    obs_t1 = PolicyObservation(
        stamp_sec=2.0,
        joint_names=("joint_1",),
        positions=(0.0,),
        velocities=(0.0,),
        efforts=(0.0,),
    )

    first_command = policy.compute_command(obs_t0)
    feedback_state["observation"] = PolicyObservation(
        stamp_sec=2.0,
        joint_names=("joint_1",),
        positions=(0.0,),
        velocities=(0.0,),
        efforts=(0.0,),
    )
    second_command = policy.compute_command(obs_t1)

    first_tip = fk.compute_tip_position(obs_t0.joint_names, first_command)
    second_tip = fk.compute_tip_position(obs_t1.joint_names, second_command)

    assert second_command[0] > first_command[0]
    assert second_tip[1] > first_tip[1]
    assert policy.debug_state()["target_tcp_xyz"][1] > policy.debug_state()["feedback_tcp_xyz"][1]


def test_cartesian_true_feedback_servo_policy_respects_saturated_error_without_runaway():
    fk = UrdfForwardKinematics.from_robot_description(
        SIMPLE_URDF,
        base_link="base_link",
        tip_link="tcp",
    )
    feedback_state = {
        "observation": PolicyObservation(
            stamp_sec=1.0,
            joint_names=("joint_1",),
            positions=(0.0,),
            velocities=(0.0,),
            efforts=(0.0,),
        )
    }
    policy = CartesianTrueFeedbackServoPolicy(
        forward_kinematics=fk,
        feedback_observation_provider=lambda: feedback_state["observation"],
        direction_xyz=(0.0, 1.0, 0.0),
        speed_mps=0.001,
        tracking_gain=1.0,
        max_cartesian_error_m=0.02,
        damping=0.01,
        max_joint_delta_rad=0.1,
    )
    obs = PolicyObservation(
        stamp_sec=1.0,
        joint_names=("joint_1",),
        positions=(0.0,),
        velocities=(0.0,),
        efforts=(0.0,),
    )

    first_command = policy.compute_command(obs)
    feedback_state["observation"] = PolicyObservation(
        stamp_sec=101.0,
        joint_names=("joint_1",),
        positions=(0.0,),
        velocities=(0.0,),
        efforts=(0.0,),
    )
    second_command = policy.compute_command(obs)
    third_command = policy.compute_command(obs)

    assert first_command[0] == pytest.approx(0.0)
    assert second_command[0] > first_command[0]
    assert third_command[0] == pytest.approx(second_command[0])
    assert policy.debug_state()["limited_error_xyz"][1] == pytest.approx(0.02)
