import math

import pytest

from abb_pi0_bridge.cartesian_workspace import UrdfForwardKinematics, distance_m


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


def test_urdf_forward_kinematics_computes_tip_position():
    fk = UrdfForwardKinematics.from_robot_description(
        SIMPLE_URDF,
        base_link="base_link",
        tip_link="tcp",
    )

    assert fk.compute_tip_position(["joint_1"], [0.0]) == pytest.approx((2.0, 0.0, 0.0))
    assert fk.compute_tip_position(["joint_1"], [math.pi / 2.0]) == pytest.approx(
        (1.0, 1.0, 0.0)
    )


def test_distance_m():
    assert distance_m((0.0, 0.0, 0.0), (0.3, 0.4, 0.0)) == pytest.approx(0.5)


def test_urdf_forward_kinematics_rejects_missing_chain():
    with pytest.raises(ValueError, match="No URDF chain"):
        UrdfForwardKinematics.from_robot_description(
            SIMPLE_URDF,
            base_link="base_link",
            tip_link="missing",
        )
