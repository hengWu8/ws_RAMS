from __future__ import annotations

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET
from typing import Mapping, Sequence, Tuple


Vector3 = Tuple[float, float, float]
Matrix4 = Tuple[Tuple[float, float, float, float], ...]


@dataclass(frozen=True)
class UrdfJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: Vector3
    origin_rpy: Vector3
    axis_xyz: Vector3


class UrdfForwardKinematics:
    """Small URDF FK helper used only for the bridge's Cartesian safety guard."""

    def __init__(self, chain: Sequence[UrdfJoint], base_link: str, tip_link: str) -> None:
        if not chain:
            raise ValueError(f"No URDF joint chain found from {base_link!r} to {tip_link!r}.")
        self.chain = tuple(chain)
        self.base_link = base_link
        self.tip_link = tip_link

    @classmethod
    def from_robot_description(
        cls,
        robot_description: str,
        *,
        base_link: str,
        tip_link: str,
    ) -> "UrdfForwardKinematics":
        if not robot_description.strip():
            raise ValueError("robot_description is empty.")

        try:
            root = ET.fromstring(robot_description)
        except ET.ParseError as exc:
            raise ValueError(f"Failed to parse robot_description XML: {exc}") from exc

        joints_by_child: dict[str, UrdfJoint] = {}
        for joint_element in root.findall("joint"):
            joint = _parse_joint(joint_element)
            if joint.child in joints_by_child:
                raise ValueError(f"URDF contains multiple parent joints for child link {joint.child!r}.")
            joints_by_child[joint.child] = joint

        reversed_chain = []
        current_link = tip_link
        visited_links = set()
        while current_link != base_link:
            if current_link in visited_links:
                raise ValueError(f"Cycle detected while resolving URDF chain at link {current_link!r}.")
            visited_links.add(current_link)
            joint = joints_by_child.get(current_link)
            if joint is None:
                raise ValueError(f"No URDF chain found from {base_link!r} to {tip_link!r}.")
            reversed_chain.append(joint)
            current_link = joint.parent

        return cls(tuple(reversed(reversed_chain)), base_link=base_link, tip_link=tip_link)

    def compute_tip_position(
        self,
        joint_names: Sequence[str],
        joint_positions: Sequence[float],
    ) -> Vector3:
        if len(joint_names) != len(joint_positions):
            raise ValueError("joint_names and joint_positions must have the same length.")

        position_by_name = {
            name: float(position)
            for name, position in zip(joint_names, joint_positions)
        }

        transform = _identity()
        for joint in self.chain:
            transform = _matmul(transform, _origin_transform(joint.origin_xyz, joint.origin_rpy))
            if joint.joint_type in ("revolute", "continuous"):
                value = position_by_name.get(joint.name)
                if value is None:
                    raise ValueError(f"Missing joint value for {joint.name!r}.")
                transform = _matmul(transform, _axis_rotation_transform(joint.axis_xyz, value))
            elif joint.joint_type == "prismatic":
                value = position_by_name.get(joint.name)
                if value is None:
                    raise ValueError(f"Missing joint value for {joint.name!r}.")
                transform = _matmul(transform, _translation_transform(_scale(joint.axis_xyz, value)))
            elif joint.joint_type == "fixed":
                pass
            else:
                raise ValueError(f"Unsupported URDF joint type {joint.joint_type!r} for {joint.name!r}.")

        return (transform[0][3], transform[1][3], transform[2][3])


def distance_m(a: Vector3, b: Vector3) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def _parse_joint(joint_element: ET.Element) -> UrdfJoint:
    name = joint_element.attrib.get("name", "")
    joint_type = joint_element.attrib.get("type", "")
    parent_element = joint_element.find("parent")
    child_element = joint_element.find("child")
    if not name or not joint_type or parent_element is None or child_element is None:
        raise ValueError("URDF joint is missing name, type, parent, or child.")

    origin_element = joint_element.find("origin")
    origin_xyz = _parse_vector(origin_element.attrib.get("xyz", "0 0 0") if origin_element is not None else "0 0 0")
    origin_rpy = _parse_vector(origin_element.attrib.get("rpy", "0 0 0") if origin_element is not None else "0 0 0")

    axis_element = joint_element.find("axis")
    axis_xyz = _parse_vector(axis_element.attrib.get("xyz", "1 0 0") if axis_element is not None else "1 0 0")
    axis_xyz = _normalize(axis_xyz)

    return UrdfJoint(
        name=name,
        joint_type=joint_type,
        parent=parent_element.attrib["link"],
        child=child_element.attrib["link"],
        origin_xyz=origin_xyz,
        origin_rpy=origin_rpy,
        axis_xyz=axis_xyz,
    )


def _parse_vector(raw: str) -> Vector3:
    parts = raw.split()
    if len(parts) != 3:
        raise ValueError(f"Expected a 3-vector, got {raw!r}.")
    values = tuple(float(part) for part in parts)
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"Vector contains non-finite values: {raw!r}.")
    return values  # type: ignore[return-value]


def _normalize(vector: Vector3) -> Vector3:
    norm = math.sqrt(sum(value * value for value in vector))
    if norm <= 0.0:
        raise ValueError("URDF joint axis must be non-zero.")
    return tuple(value / norm for value in vector)  # type: ignore[return-value]


def _scale(vector: Vector3, value: float) -> Vector3:
    return tuple(float(value) * component for component in vector)  # type: ignore[return-value]


def _identity() -> Matrix4:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _translation_transform(xyz: Vector3) -> Matrix4:
    x, y, z = xyz
    return (
        (1.0, 0.0, 0.0, x),
        (0.0, 1.0, 0.0, y),
        (0.0, 0.0, 1.0, z),
        (0.0, 0.0, 0.0, 1.0),
    )


def _origin_transform(xyz: Vector3, rpy: Vector3) -> Matrix4:
    return _matmul(_translation_transform(xyz), _rpy_transform(rpy))


def _rpy_transform(rpy: Vector3) -> Matrix4:
    roll, pitch, yaw = rpy
    return _matmul(_matmul(_z_rotation(yaw), _y_rotation(pitch)), _x_rotation(roll))


def _axis_rotation_transform(axis: Vector3, angle: float) -> Matrix4:
    x, y, z = axis
    c = math.cos(float(angle))
    s = math.sin(float(angle))
    one_c = 1.0 - c
    return (
        (c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s, 0.0),
        (y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s, 0.0),
        (z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _x_rotation(angle: float) -> Matrix4:
    c = math.cos(angle)
    s = math.sin(angle)
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, c, -s, 0.0),
        (0.0, s, c, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _y_rotation(angle: float) -> Matrix4:
    c = math.cos(angle)
    s = math.sin(angle)
    return (
        (c, 0.0, s, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (-s, 0.0, c, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _z_rotation(angle: float) -> Matrix4:
    c = math.cos(angle)
    s = math.sin(angle)
    return (
        (c, -s, 0.0, 0.0),
        (s, c, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _matmul(a: Matrix4, b: Matrix4) -> Matrix4:
    rows = []
    for row in range(4):
        values = []
        for col in range(4):
            values.append(sum(a[row][k] * b[k][col] for k in range(4)))
        rows.append(tuple(values))
    return tuple(rows)  # type: ignore[return-value]
