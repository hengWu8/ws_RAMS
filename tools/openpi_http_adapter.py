#!/usr/bin/env python3
"""Minimal HTTP adapter between abb_pi0_bridge and an openpi checkpoint.

This tool is intentionally conservative:
- It accepts the current ABB bridge `/infer` JSON schema.
- It builds a minimal openpi observation using joint positions as low-dimensional state.
- It uses zero images plus a fixed prompt for initial dry-run testing.
- It maps the first predicted action step into small joint deltas around the current ABB state.

This is suitable for validating the request/response path with a real model before wiring
real camera images or robot-specific policy inputs into the stack.
"""

from __future__ import annotations

import argparse
import dataclasses
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
import math
from pathlib import Path
import shlex
import subprocess
import sys
import threading
from typing import Any

import cv2
import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
WS_RAMS_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OPENPI_ROOT = WORKSPACE_ROOT / "openpi_official"
OPENPI_SRC = DEFAULT_OPENPI_ROOT / "src"
OPENPI_CLIENT_SRC = DEFAULT_OPENPI_ROOT / "packages" / "openpi-client" / "src"
DEFAULT_WORKCELL_XACRO = (
    WS_RAMS_ROOT / "src" / "workcell_description" / "urdf" / "workcell.urdf.xacro"
)

for candidate in (OPENPI_SRC, OPENPI_CLIENT_SRC):
    candidate_str = str(candidate)
    if candidate.exists() and candidate_str not in sys.path:
        sys.path.insert(0, candidate_str)


@dataclasses.dataclass(frozen=True)
class JointSpec:
    name: str
    parent: str
    child: str
    joint_type: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray


@dataclasses.dataclass(frozen=True)
class KinematicSnapshot:
    tcp_transform: np.ndarray
    movable_points_world: tuple[np.ndarray, ...]
    base_position_world: np.ndarray


@dataclasses.dataclass(frozen=True)
class SceneObject:
    name: str
    offset_xyz: np.ndarray
    half_extents_xyz: np.ndarray
    color_bgr: tuple[int, int, int]


def _rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def _transform_from_xyz_rpy(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = _rotation_matrix_from_rpy(*rpy)
    transform[:3, 3] = xyz
    return transform


def _axis_angle_to_rotation_matrix(axis_angle: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(axis_angle))
    if theta < 1e-9:
        return np.eye(3, dtype=np.float64)

    axis = axis_angle / theta
    x, y, z = axis
    skew = np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=np.float64,
    )
    identity = np.eye(3, dtype=np.float64)
    return identity + math.sin(theta) * skew + (1.0 - math.cos(theta)) * (skew @ skew)


def _rotation_matrix_to_axis_angle(rotation: np.ndarray) -> np.ndarray:
    trace_value = float(np.trace(rotation))
    cos_theta = max(-1.0, min(1.0, (trace_value - 1.0) * 0.5))
    theta = math.acos(cos_theta)
    if theta < 1e-9:
        return np.zeros(3, dtype=np.float64)

    sin_theta = math.sin(theta)
    if abs(sin_theta) < 1e-6:
        diagonal = np.clip((np.diag(rotation) + 1.0) * 0.5, 0.0, None)
        axis = np.sqrt(diagonal)
        if rotation[2, 1] - rotation[1, 2] < 0.0:
            axis[0] = -axis[0]
        if rotation[0, 2] - rotation[2, 0] < 0.0:
            axis[1] = -axis[1]
        if rotation[1, 0] - rotation[0, 1] < 0.0:
            axis[2] = -axis[2]
        norm = np.linalg.norm(axis)
        if norm < 1e-9:
            return np.zeros(3, dtype=np.float64)
        return axis / norm * theta

    axis = np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ],
        dtype=np.float64,
    ) / (2.0 * sin_theta)
    return axis * theta


def _rotation_matrix_from_axis_angle_xyz(rx: float, ry: float, rz: float) -> np.ndarray:
    return _axis_angle_to_rotation_matrix(np.array([rx, ry, rz], dtype=np.float64))


def _rotation_about_axis(axis: np.ndarray, angle: float) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = _axis_angle_to_rotation_matrix(np.asarray(axis, dtype=np.float64) * angle)
    return transform


class AbbLiberoMapper:
    def __init__(
        self,
        *,
        workcell_xacro: str,
        base_link: str,
        tip_link: str,
        joint_names: tuple[str, ...],
        translation_scale_m: float,
        rotation_scale_rad: float,
        ik_damping: float,
        max_joint_step_rad: float,
    ) -> None:
        self.workcell_xacro = workcell_xacro
        self.base_link = base_link
        self.tip_link = tip_link
        self.joint_names = joint_names
        self.translation_scale_m = translation_scale_m
        self.rotation_scale_rad = rotation_scale_rad
        self.ik_damping = ik_damping
        self.max_joint_step_rad = max_joint_step_rad
        self._chain = self._load_chain()

    def state_from_joint_positions(self, current_positions: np.ndarray, *, state_dim: int) -> np.ndarray:
        snapshot, _ = self._forward_kinematics_and_jacobian(current_positions)
        tcp_position = snapshot.tcp_transform[:3, 3]
        tcp_axis_angle = _rotation_matrix_to_axis_angle(snapshot.tcp_transform[:3, :3])

        state = np.zeros(state_dim, dtype=np.float32)
        packed = np.concatenate(
            [tcp_position.astype(np.float32), tcp_axis_angle.astype(np.float32), np.zeros(1, dtype=np.float32)]
        )
        copy_count = min(state_dim, packed.size)
        state[:copy_count] = packed[:copy_count]
        return state

    def map_actions_to_joint_positions(self, current_positions: np.ndarray, actions: np.ndarray) -> list[float]:
        actions = np.asarray(actions, dtype=np.float32)
        if actions.ndim != 2 or actions.shape[0] == 0:
            raise ValueError("openpi actions must have shape [action_horizon, action_dim].")
        first_action = actions[0]
        if first_action.size < 6:
            raise ValueError(
                f"openpi action dimension {first_action.size} is too small for ABB cartesian mapping."
            )

        _snapshot, jacobian = self._forward_kinematics_and_jacobian(current_positions)

        translation_delta = np.tanh(first_action[:3].astype(np.float64)) * self.translation_scale_m
        rotation_delta = np.tanh(first_action[3:6].astype(np.float64)) * self.rotation_scale_rad
        cartesian_delta = np.concatenate([translation_delta, rotation_delta])

        regularizer = (self.ik_damping ** 2) * np.eye(6, dtype=np.float64)
        damped_inverse = jacobian.T @ np.linalg.inv(jacobian @ jacobian.T + regularizer)
        joint_delta = damped_inverse @ cartesian_delta
        joint_delta = np.clip(joint_delta, -self.max_joint_step_rad, self.max_joint_step_rad)

        return (current_positions[: len(self.joint_names)] + joint_delta).astype(float).tolist()

    def snapshot(self, current_positions: np.ndarray) -> KinematicSnapshot:
        snapshot, _ = self._forward_kinematics_and_jacobian(current_positions)
        return snapshot

    def _load_chain(self) -> list[JointSpec]:
        urdf_xml = self._expand_xacro()
        import xml.etree.ElementTree as element_tree

        root = element_tree.fromstring(urdf_xml)
        child_to_joint: dict[str, JointSpec] = {}
        for joint_element in root.findall("joint"):
            joint_type = joint_element.attrib["type"]
            parent = joint_element.find("parent").attrib["link"]
            child = joint_element.find("child").attrib["link"]
            origin_element = joint_element.find("origin")
            xyz = np.zeros(3, dtype=np.float64)
            rpy = np.zeros(3, dtype=np.float64)
            if origin_element is not None:
                xyz = np.fromstring(origin_element.attrib.get("xyz", "0 0 0"), sep=" ", dtype=np.float64)
                rpy = np.fromstring(origin_element.attrib.get("rpy", "0 0 0"), sep=" ", dtype=np.float64)

            axis_element = joint_element.find("axis")
            axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
            if axis_element is not None:
                axis = np.fromstring(axis_element.attrib.get("xyz", "1 0 0"), sep=" ", dtype=np.float64)
            norm = np.linalg.norm(axis)
            if norm > 1e-9:
                axis = axis / norm

            spec = JointSpec(
                name=joint_element.attrib["name"],
                parent=parent,
                child=child,
                joint_type=joint_type,
                origin_xyz=xyz,
                origin_rpy=rpy,
                axis=axis,
            )
            child_to_joint[child] = spec

        chain: list[JointSpec] = []
        current_link = self.tip_link
        while current_link != self.base_link:
            if current_link not in child_to_joint:
                raise ValueError(
                    f"Could not find a joint chain from base_link='{self.base_link}' to tip_link='{self.tip_link}'."
                )
            spec = child_to_joint[current_link]
            chain.append(spec)
            current_link = spec.parent
        chain.reverse()
        movable_joint_count = sum(
            1 for spec in chain if spec.joint_type in {"revolute", "continuous", "prismatic"}
        )
        if movable_joint_count != len(self.joint_names):
            raise ValueError(
                f"ABB mapper expected {len(self.joint_names)} movable joints but found {movable_joint_count}."
            )
        return chain

    def _expand_xacro(self) -> str:
        command = " ".join(
            [
                "source /opt/ros/humble/setup.bash",
                "&&",
                f"source {shlex.quote(str(WS_RAMS_ROOT / 'install' / 'setup.bash'))}",
                "&&",
                "xacro",
                shlex.quote(self.workcell_xacro),
                "use_fake_hardware:=false",
                "rws_ip:=127.0.0.1",
                "rws_port:=28080",
                "egm_port:=6515",
            ]
        )
        result = subprocess.run(
            ["bash", "-lc", command],
            check=True,
            capture_output=True,
            text=True,
        )
        return result.stdout

    def _forward_kinematics_and_jacobian(self, current_positions: np.ndarray) -> tuple[KinematicSnapshot, np.ndarray]:
        if current_positions.size < len(self.joint_names):
            raise ValueError("joint_positions must contain ABB joint values for the configured chain.")

        transform = np.eye(4, dtype=np.float64)
        joint_origins: list[np.ndarray] = []
        joint_axes: list[np.ndarray] = []
        movable_points_world: list[np.ndarray] = [transform[:3, 3].copy()]
        joint_values = iter(current_positions[: len(self.joint_names)])
        for spec in self._chain:
            transform = transform @ _transform_from_xyz_rpy(spec.origin_xyz, spec.origin_rpy)
            if spec.joint_type in {"revolute", "continuous"}:
                joint_value = float(next(joint_values))
                joint_origins.append(transform[:3, 3].copy())
                joint_axes.append(transform[:3, :3] @ spec.axis)
                transform = transform @ _rotation_about_axis(spec.axis, joint_value)
                movable_points_world.append(transform[:3, 3].copy())
            elif spec.joint_type == "fixed":
                continue
            else:
                raise ValueError(f"Unsupported joint type '{spec.joint_type}' in ABB mapper.")

        tcp_position = transform[:3, 3]
        movable_points_world.append(tcp_position.copy())
        jacobian = np.zeros((6, len(joint_axes)), dtype=np.float64)
        for index, (origin, axis_world) in enumerate(zip(joint_origins, joint_axes, strict=True)):
            jacobian[:3, index] = np.cross(axis_world, tcp_position - origin)
            jacobian[3:, index] = axis_world

        snapshot = KinematicSnapshot(
            tcp_transform=transform,
            movable_points_world=tuple(movable_points_world),
            base_position_world=movable_points_world[0],
        )
        return snapshot, jacobian


class SyntheticVisionRenderer:
    def __init__(
        self,
        *,
        image_size: int,
        enabled: bool,
        table_extent_xy: tuple[float, float],
        object_offsets: tuple[np.ndarray, ...],
    ) -> None:
        self.image_size = image_size
        self.enabled = enabled
        self.table_extent_xy = table_extent_xy
        self._anchor_lock = threading.Lock()
        self._anchor_tcp_position: np.ndarray | None = None
        self._scene_objects = tuple(
            SceneObject(
                name=f"cube_{index}",
                offset_xyz=offset.astype(np.float64),
                half_extents_xyz=np.array([0.035, 0.035, 0.035], dtype=np.float64),
                color_bgr=color,
            )
            for index, (offset, color) in enumerate(
                zip(
                    object_offsets,
                    (
                        (40, 90, 220),
                        (60, 180, 100),
                        (200, 160, 40),
                    ),
                    strict=False,
                )
            )
        )

    def render(
        self,
        *,
        snapshot: KinematicSnapshot | None,
        prompt: str,
    ) -> tuple[np.ndarray, np.ndarray]:
        if not self.enabled or snapshot is None:
            zeros = np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8)
            return zeros, zeros

        anchor = self._get_or_init_anchor(snapshot.tcp_transform[:3, 3])
        object_positions = [anchor + scene_object.offset_xyz for scene_object in self._scene_objects]
        front_image = self._render_workspace_view(snapshot, object_positions, prompt)
        wrist_image = self._render_wrist_view(snapshot, object_positions, prompt)
        return front_image, wrist_image

    def _get_or_init_anchor(self, tcp_position: np.ndarray) -> np.ndarray:
        with self._anchor_lock:
            if self._anchor_tcp_position is None:
                self._anchor_tcp_position = tcp_position.astype(np.float64).copy()
            return self._anchor_tcp_position.copy()

    def _render_workspace_view(
        self,
        snapshot: KinematicSnapshot,
        object_positions: list[np.ndarray],
        prompt: str,
    ) -> np.ndarray:
        image = np.full((self.image_size, self.image_size, 3), (236, 240, 242), dtype=np.uint8)
        image[: int(self.image_size * 0.35), :] = (214, 228, 238)

        tcp_position = snapshot.tcp_transform[:3, 3]
        world_center_xy = np.array([tcp_position[0], tcp_position[1]], dtype=np.float64)
        extent_x, extent_y = self.table_extent_xy
        min_xy = world_center_xy - np.array([extent_x * 0.5, extent_y * 0.5], dtype=np.float64)
        max_xy = world_center_xy + np.array([extent_x * 0.5, extent_y * 0.5], dtype=np.float64)

        def project_xy(point_xyz: np.ndarray) -> tuple[int, int]:
            px = int(np.clip((point_xyz[0] - min_xy[0]) / max(extent_x, 1e-6) * (self.image_size - 1), 0, self.image_size - 1))
            py = int(
                np.clip(
                    (1.0 - (point_xyz[1] - min_xy[1]) / max(extent_y, 1e-6)) * (self.image_size - 1),
                    0,
                    self.image_size - 1,
                )
            )
            return px, py

        grid_color = (210, 214, 218)
        for grid_index in range(6):
            x = int(grid_index * (self.image_size - 1) / 5)
            y = int(grid_index * (self.image_size - 1) / 5)
            cv2.line(image, (x, 0), (x, self.image_size - 1), grid_color, 1, cv2.LINE_AA)
            cv2.line(image, (0, y), (self.image_size - 1, y), grid_color, 1, cv2.LINE_AA)

        for scene_object, object_position in zip(self._scene_objects, object_positions, strict=True):
            center = project_xy(object_position)
            half_size_px = max(8, int(0.04 / max(extent_x, 1e-6) * self.image_size))
            top_left = (center[0] - half_size_px, center[1] - half_size_px)
            bottom_right = (center[0] + half_size_px, center[1] + half_size_px)
            cv2.rectangle(image, top_left, bottom_right, scene_object.color_bgr, -1, cv2.LINE_AA)
            cv2.rectangle(image, top_left, bottom_right, (40, 40, 40), 2, cv2.LINE_AA)

        projected_points = [project_xy(point) for point in snapshot.movable_points_world]
        for start, end in zip(projected_points[:-1], projected_points[1:], strict=True):
            cv2.line(image, start, end, (42, 50, 64), 6, cv2.LINE_AA)
        for projected_point in projected_points[:-1]:
            cv2.circle(image, projected_point, 6, (240, 244, 248), -1, cv2.LINE_AA)
            cv2.circle(image, projected_point, 2, (42, 50, 64), -1, cv2.LINE_AA)
        cv2.circle(image, projected_points[-1], 8, (24, 90, 220), -1, cv2.LINE_AA)

        self._draw_prompt_banner(image, prompt, header="front")
        return image

    def _render_wrist_view(
        self,
        snapshot: KinematicSnapshot,
        object_positions: list[np.ndarray],
        prompt: str,
    ) -> np.ndarray:
        image = np.full((self.image_size, self.image_size, 3), (28, 34, 40), dtype=np.uint8)
        tcp_transform = snapshot.tcp_transform
        tcp_position = tcp_transform[:3, 3]
        rotation_world_tcp = tcp_transform[:3, :3]
        rotation_tcp_world = rotation_world_tcp.T

        visible_objects: list[tuple[SceneObject, np.ndarray]] = []
        for scene_object, object_position in zip(self._scene_objects, object_positions, strict=True):
            relative_tcp = rotation_tcp_world @ (object_position - tcp_position)
            visible_objects.append((scene_object, relative_tcp))

        visible_objects.sort(key=lambda item: item[1][0], reverse=True)
        for scene_object, relative_tcp in visible_objects:
            depth = float(relative_tcp[0] + 0.35)
            lateral = float(relative_tcp[1])
            vertical = float(relative_tcp[2])
            if depth <= 0.03:
                continue
            x_px = int(self.image_size * 0.5 + lateral / max(depth, 1e-6) * self.image_size * 0.7)
            y_px = int(self.image_size * 0.62 - vertical / max(depth, 1e-6) * self.image_size * 0.7)
            size_px = int(np.clip(26.0 / depth, 12, self.image_size * 0.35))
            if x_px + size_px < 0 or x_px - size_px >= self.image_size or y_px + size_px < 0 or y_px - size_px >= self.image_size:
                continue
            top_left = (x_px - size_px, y_px - size_px)
            bottom_right = (x_px + size_px, y_px + size_px)
            cv2.rectangle(image, top_left, bottom_right, scene_object.color_bgr, -1, cv2.LINE_AA)
            cv2.rectangle(image, top_left, bottom_right, (235, 240, 245), 2, cv2.LINE_AA)

        center = self.image_size // 2
        cv2.line(image, (center - 10, center), (center + 10, center), (245, 245, 245), 1, cv2.LINE_AA)
        cv2.line(image, (center, center - 10), (center, center + 10), (245, 245, 245), 1, cv2.LINE_AA)
        cv2.circle(image, (center, center), 3, (245, 245, 245), -1, cv2.LINE_AA)
        self._draw_prompt_banner(image, prompt, header="wrist")
        return image

    def _draw_prompt_banner(self, image: np.ndarray, prompt: str, *, header: str) -> None:
        cv2.rectangle(image, (0, 0), (self.image_size - 1, 26), (245, 248, 250), -1, cv2.LINE_AA)
        label = f"{header}: {prompt[:28]}"
        cv2.putText(
            image,
            label,
            (8, 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (30, 36, 44),
            1,
            cv2.LINE_AA,
        )


def build_openpi_observation(
    request_payload: dict[str, Any],
    *,
    mapper: AbbLiberoMapper | None,
    renderer: SyntheticVisionRenderer | None,
    state_dim: int,
    image_size: int,
    prompt: str,
) -> tuple[dict[str, Any], np.ndarray]:
    observation = request_payload.get("observation", {})
    current_positions = np.asarray(observation.get("joint_positions", []), dtype=np.float32)
    if current_positions.ndim != 1 or current_positions.size == 0:
        raise ValueError("observation.joint_positions must be a non-empty 1D vector.")

    snapshot: KinematicSnapshot | None = None
    if mapper is None:
        state = np.zeros(state_dim, dtype=np.float32)
        copy_count = min(state_dim, current_positions.size)
        state[:copy_count] = current_positions[:copy_count]
    else:
        snapshot = mapper.snapshot(current_positions)
        state = mapper.state_from_joint_positions(current_positions, state_dim=state_dim)

    request_prompt = str(request_payload.get("prompt", prompt))
    if renderer is None:
        zeros_image = np.zeros((image_size, image_size, 3), dtype=np.uint8)
        image = zeros_image
        wrist_image = zeros_image
    else:
        image, wrist_image = renderer.render(snapshot=snapshot, prompt=request_prompt)

    openpi_observation = {
        "observation/state": state,
        "observation/image": image,
        "observation/wrist_image": wrist_image,
        "prompt": request_prompt,
    }
    return openpi_observation, current_positions


def map_openpi_actions_to_joint_positions(
    current_positions: np.ndarray,
    actions: np.ndarray,
    *,
    mapper: AbbLiberoMapper | None,
    joint_count: int,
    delta_scale_rad: float,
) -> list[float]:
    if mapper is not None:
        return mapper.map_actions_to_joint_positions(current_positions, actions)

    actions = np.asarray(actions, dtype=np.float32)
    if actions.ndim != 2 or actions.shape[0] == 0:
        raise ValueError("openpi actions must have shape [action_horizon, action_dim].")
    if delta_scale_rad <= 0.0:
        raise ValueError("delta_scale_rad must be positive.")

    first_action = actions[0]
    if first_action.size < joint_count:
        raise ValueError(
            f"openpi action dimension {first_action.size} is smaller than joint_count {joint_count}."
        )

    deltas = np.tanh(first_action[:joint_count]) * float(delta_scale_rad)
    return (current_positions[:joint_count] + deltas).astype(float).tolist()


class OpenPiPolicyAdapter:
    def __init__(
        self,
        *,
        config_name: str,
        checkpoint_dir: str,
        prompt: str,
        state_dim: int,
        image_size: int,
        joint_count: int,
        delta_scale_rad: float,
        pytorch_device: str,
        mapping_mode: str,
        workcell_xacro: str,
        base_link: str,
        tip_link: str,
        translation_scale_m: float,
        rotation_scale_rad: float,
        ik_damping: float,
        max_joint_step_rad: float,
        use_synthetic_vision: bool,
        table_extent_xy: tuple[float, float],
        object_offsets: tuple[np.ndarray, ...],
    ) -> None:
        self.config_name = config_name
        self.checkpoint_dir = checkpoint_dir
        self.prompt = prompt
        self.state_dim = state_dim
        self.image_size = image_size
        self.joint_count = joint_count
        self.delta_scale_rad = delta_scale_rad
        self.pytorch_device = pytorch_device
        self.mapping_mode = mapping_mode
        self.mapper = (
            AbbLiberoMapper(
                workcell_xacro=workcell_xacro,
                base_link=base_link,
                tip_link=tip_link,
                joint_names=tuple(f"joint_{index}" for index in range(1, joint_count + 1)),
                translation_scale_m=translation_scale_m,
                rotation_scale_rad=rotation_scale_rad,
                ik_damping=ik_damping,
                max_joint_step_rad=max_joint_step_rad,
            )
            if mapping_mode == "abb_libero"
            else None
        )
        self.renderer = SyntheticVisionRenderer(
            image_size=image_size,
            enabled=use_synthetic_vision,
            table_extent_xy=table_extent_xy,
            object_offsets=object_offsets,
        )

        self._policy = None
        self._policy_lock = threading.Lock()
        self._load_error: str | None = None

    def health_payload(self) -> dict[str, Any]:
        return {
            "ok": self._load_error is None,
            "policy_loaded": self._policy is not None,
            "load_error": self._load_error,
            "config_name": self.config_name,
            "checkpoint_dir": self.checkpoint_dir,
            "prompt": self.prompt,
            "state_dim": self.state_dim,
            "image_size": self.image_size,
            "joint_count": self.joint_count,
            "delta_scale_rad": self.delta_scale_rad,
            "pytorch_device": self.pytorch_device,
            "mapping_mode": self.mapping_mode,
            "tcp_tip_link": self.mapper.tip_link if self.mapper is not None else None,
            "synthetic_vision": self.renderer.enabled,
        }

    def infer(self, request_payload: dict[str, Any]) -> dict[str, Any]:
        policy = self._get_policy()
        openpi_observation, current_positions = build_openpi_observation(
            request_payload,
            mapper=self.mapper,
            renderer=self.renderer,
            state_dim=self.state_dim,
            image_size=self.image_size,
            prompt=self.prompt,
        )
        result = policy.infer(openpi_observation)
        joint_positions = map_openpi_actions_to_joint_positions(
            current_positions=current_positions,
            actions=np.asarray(result["actions"]),
            mapper=self.mapper,
            joint_count=self.joint_count,
            delta_scale_rad=self.delta_scale_rad,
        )
        return {
            "joint_positions": joint_positions,
            "adapter": {
                "backend": "openpi",
                "config_name": self.config_name,
                "checkpoint_dir": self.checkpoint_dir,
                "prompt": self.prompt,
                "state_dim": self.state_dim,
                "image_size": self.image_size,
                "joint_count": self.joint_count,
                "delta_scale_rad": self.delta_scale_rad,
                "mapping_mode": self.mapping_mode,
            },
            "policy_timing": result.get("policy_timing"),
        }

    def _get_policy(self):
        if self._policy is not None:
            return self._policy

        with self._policy_lock:
            if self._policy is not None:
                return self._policy

            try:
                import torch

                from openpi.policies import policy as openpi_policy
                from openpi.shared import download as openpi_download
                from openpi.shared import normalize as openpi_normalize
                from openpi.training import config as openpi_config
                import openpi.transforms as openpi_transforms

                train_cfg = openpi_config.get_config(self.config_name)
                train_cfg = dataclasses.replace(
                    train_cfg,
                    model=dataclasses.replace(train_cfg.model, pytorch_compile_mode=None),
                )
                checkpoint_dir = openpi_download.maybe_download(str(self.checkpoint_dir))
                checkpoint_dir = Path(checkpoint_dir)
                pytorch_device = _resolve_pytorch_device(self.pytorch_device, torch)

                model = train_cfg.model.load_pytorch(
                    train_cfg,
                    str(checkpoint_dir / "model.safetensors"),
                    device=pytorch_device,
                )
                model.paligemma_with_expert.to_bfloat16_for_selected_params("bfloat16")

                data_config = train_cfg.data.create(train_cfg.assets_dirs, train_cfg.model)
                if data_config.asset_id is None:
                    raise ValueError("Asset id is required to load norm stats for ABB adapter inference.")

                norm_stats = openpi_normalize.load(checkpoint_dir / "assets" / data_config.asset_id)
                self._policy = openpi_policy.Policy(
                    model,
                    transforms=[
                        openpi_transforms.InjectDefaultPrompt(self.prompt),
                        *data_config.data_transforms.inputs,
                        openpi_transforms.Normalize(norm_stats, use_quantiles=data_config.use_quantile_norm),
                        *data_config.model_transforms.inputs,
                    ],
                    output_transforms=[
                        *data_config.model_transforms.outputs,
                        openpi_transforms.Unnormalize(norm_stats, use_quantiles=data_config.use_quantile_norm),
                        *data_config.data_transforms.outputs,
                    ],
                    metadata=train_cfg.policy_metadata,
                    is_pytorch=True,
                    pytorch_device=pytorch_device,
                )
                logging.info(
                    "Loaded openpi policy config=%s checkpoint=%s device=%s",
                    self.config_name,
                    checkpoint_dir,
                    pytorch_device,
                )
            except Exception as exc:  # pragma: no cover - surfaced via HTTP and logs
                self._load_error = str(exc)
                logging.exception("Failed to load openpi policy")
                raise

            return self._policy


class _AdapterHandler(BaseHTTPRequestHandler):
    server_version = "OpenPiAbbHttpAdapter/0.1"

    @property
    def adapter(self) -> OpenPiPolicyAdapter:
        return getattr(self.server, "adapter")

    def do_GET(self) -> None:
        if self.path != "/healthz":
            self._write_json(404, {"error": "not_found"})
            return
        self._write_json(200, self.adapter.health_payload())

    def do_POST(self) -> None:
        if self.path != "/infer":
            self._write_json(404, {"error": "not_found"})
            return

        content_length = int(self.headers.get("Content-Length", "0"))
        raw_body = self.rfile.read(content_length).decode("utf-8")

        try:
            request_payload = json.loads(raw_body) if raw_body else {}
        except json.JSONDecodeError:
            self._write_json(400, {"error": "invalid_json"})
            return

        try:
            response_payload = self.adapter.infer(request_payload)
        except Exception as exc:
            self._write_json(
                500,
                {
                    "error": str(exc),
                    "health": self.adapter.health_payload(),
                },
            )
            return

        self._write_json(200, response_payload)

    def log_message(self, format: str, *args) -> None:  # noqa: A003
        logging.info("%s - %s", self.address_string(), format % args)

    def _write_json(self, status_code: int, payload: dict[str, Any]) -> None:
        response = json.dumps(payload).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(response)))
        self.end_headers()
        self.wfile.write(response)


def _default_checkpoint_dir() -> str:
    return str(DEFAULT_OPENPI_ROOT / "checkpoints" / "pi05_libero_pytorch")


def _resolve_pytorch_device(requested_device: str, torch_module) -> str:
    if requested_device != "auto":
        return requested_device
    if not torch_module.cuda.is_available():
        return "cpu"

    best_device = "cuda:0"
    most_free_bytes = -1
    for device_index in range(torch_module.cuda.device_count()):
        free_bytes, _ = torch_module.cuda.mem_get_info(device_index)
        if free_bytes > most_free_bytes:
            most_free_bytes = free_bytes
            best_device = f"cuda:{device_index}"
    return best_device


def _parse_object_offsets(raw_value: str) -> tuple[np.ndarray, ...]:
    offsets: list[np.ndarray] = []
    for item in raw_value.split(";"):
        stripped = item.strip()
        if not stripped:
            continue
        values = np.fromstring(stripped, sep=",", dtype=np.float64)
        if values.size != 3:
            raise ValueError(
                "Each synthetic object offset must contain exactly 3 comma-separated values, "
                f"got '{stripped}'."
            )
        offsets.append(values)
    if not offsets:
        raise ValueError("At least one synthetic object offset must be provided.")
    return tuple(offsets)


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Minimal ABB HTTP to openpi adapter for safe dry-run testing."
    )
    parser.add_argument("--host", default="127.0.0.1", help="Bind address for the adapter.")
    parser.add_argument("--port", type=int, default=8001, help="Bind port for the adapter.")
    parser.add_argument(
        "--config-name",
        default="pi05_libero",
        help="openpi training config to load, e.g. pi05_libero or pi05_droid.",
    )
    parser.add_argument(
        "--checkpoint-dir",
        default=_default_checkpoint_dir(),
        help="Path or URI of the openpi checkpoint.",
    )
    parser.add_argument(
        "--prompt",
        default="move left",
        help="Default prompt to send to openpi when the ABB request does not include one.",
    )
    parser.add_argument(
        "--state-dim",
        type=int,
        default=8,
        help="openpi state vector size used by the selected config.",
    )
    parser.add_argument(
        "--image-size",
        type=int,
        default=224,
        help="Square image size used for zero-image placeholders.",
    )
    parser.add_argument(
        "--joint-count",
        type=int,
        default=6,
        help="Number of ABB joints to control.",
    )
    parser.add_argument(
        "--delta-scale-rad",
        type=float,
        default=0.05,
        help="Scale applied to the first openpi action step before adding it to current joints.",
    )
    parser.add_argument(
        "--pytorch-device",
        default="auto",
        help="Device for the PyTorch checkpoint, e.g. auto, cpu, cuda, cuda:0.",
    )
    parser.add_argument(
        "--mapping-mode",
        choices=("abb_libero", "joint_delta"),
        default="abb_libero",
        help="How to interpret openpi actions before returning ABB joint targets.",
    )
    parser.add_argument(
        "--workcell-xacro",
        default=str(DEFAULT_WORKCELL_XACRO),
        help="Path to the workcell xacro used to build ABB TCP kinematics.",
    )
    parser.add_argument(
        "--base-link",
        default="base_link",
        help="Base link for ABB differential IK.",
    )
    parser.add_argument(
        "--tip-link",
        default="tcp",
        help="TCP link used for ABB differential IK.",
    )
    parser.add_argument(
        "--translation-scale-m",
        type=float,
        default=0.05,
        help="Meters mapped from the first 3 LIBERO action dimensions to ABB TCP translation deltas.",
    )
    parser.add_argument(
        "--rotation-scale-rad",
        type=float,
        default=0.20,
        help="Radians mapped from LIBERO orientation deltas to ABB TCP orientation deltas.",
    )
    parser.add_argument(
        "--ik-damping",
        type=float,
        default=0.05,
        help="Damping factor for ABB Jacobian pseudo-inverse IK.",
    )
    parser.add_argument(
        "--max-joint-step-rad",
        type=float,
        default=0.10,
        help="Maximum per-step ABB joint delta produced by the adapter IK layer.",
    )
    parser.add_argument(
        "--use-synthetic-vision",
        action="store_true",
        help="Render two synthetic RGB views from ABB state instead of zero-image placeholders.",
    )
    parser.add_argument(
        "--table-extent-xy",
        default="0.9,0.9",
        help="Workspace view extent in meters for the synthetic front image, formatted as x_extent,y_extent.",
    )
    parser.add_argument(
        "--synthetic-object-offsets",
        default="0.18,0.12,-0.03;0.26,-0.08,-0.03",
        help=(
            "Semicolon-separated object offsets relative to the first observed TCP position, "
            "each formatted as x,y,z in meters."
        ),
    )
    parsed = parser.parse_args(args=args)

    logging.basicConfig(level=logging.INFO, force=True)
    table_extent_xy = np.fromstring(parsed.table_extent_xy, sep=",", dtype=np.float64)
    if table_extent_xy.size != 2:
        raise ValueError("--table-extent-xy must contain exactly 2 comma-separated values.")
    object_offsets = _parse_object_offsets(parsed.synthetic_object_offsets)

    adapter = OpenPiPolicyAdapter(
        config_name=parsed.config_name,
        checkpoint_dir=parsed.checkpoint_dir,
        prompt=parsed.prompt,
        state_dim=parsed.state_dim,
        image_size=parsed.image_size,
        joint_count=parsed.joint_count,
        delta_scale_rad=parsed.delta_scale_rad,
        pytorch_device=parsed.pytorch_device,
        mapping_mode=parsed.mapping_mode,
        workcell_xacro=parsed.workcell_xacro,
        base_link=parsed.base_link,
        tip_link=parsed.tip_link,
        translation_scale_m=parsed.translation_scale_m,
        rotation_scale_rad=parsed.rotation_scale_rad,
        ik_damping=parsed.ik_damping,
        max_joint_step_rad=parsed.max_joint_step_rad,
        use_synthetic_vision=parsed.use_synthetic_vision,
        table_extent_xy=(float(table_extent_xy[0]), float(table_extent_xy[1])),
        object_offsets=object_offsets,
    )
    server = ThreadingHTTPServer((parsed.host, parsed.port), _AdapterHandler)
    server.adapter = adapter
    logging.info("openpi ABB HTTP adapter listening on http://%s:%d", parsed.host, parsed.port)
    logging.info("health endpoint: http://%s:%d/healthz", parsed.host, parsed.port)
    logging.info("infer endpoint: http://%s:%d/infer", parsed.host, parsed.port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
