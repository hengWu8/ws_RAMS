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
        tcp_transform, _ = self._forward_kinematics_and_jacobian(current_positions)
        tcp_position = tcp_transform[:3, 3]
        tcp_axis_angle = _rotation_matrix_to_axis_angle(tcp_transform[:3, :3])

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

        tcp_transform, jacobian = self._forward_kinematics_and_jacobian(current_positions)
        del tcp_transform  # Jacobian already captures the current differential geometry.

        translation_delta = np.tanh(first_action[:3].astype(np.float64)) * self.translation_scale_m
        rotation_delta = np.tanh(first_action[3:6].astype(np.float64)) * self.rotation_scale_rad
        cartesian_delta = np.concatenate([translation_delta, rotation_delta])

        regularizer = (self.ik_damping ** 2) * np.eye(6, dtype=np.float64)
        damped_inverse = jacobian.T @ np.linalg.inv(jacobian @ jacobian.T + regularizer)
        joint_delta = damped_inverse @ cartesian_delta
        joint_delta = np.clip(joint_delta, -self.max_joint_step_rad, self.max_joint_step_rad)

        return (current_positions[: len(self.joint_names)] + joint_delta).astype(float).tolist()

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

    def _forward_kinematics_and_jacobian(self, current_positions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if current_positions.size < len(self.joint_names):
            raise ValueError("joint_positions must contain ABB joint values for the configured chain.")

        transform = np.eye(4, dtype=np.float64)
        joint_origins: list[np.ndarray] = []
        joint_axes: list[np.ndarray] = []
        joint_values = iter(current_positions[: len(self.joint_names)])
        for spec in self._chain:
            transform = transform @ _transform_from_xyz_rpy(spec.origin_xyz, spec.origin_rpy)
            if spec.joint_type in {"revolute", "continuous"}:
                joint_value = float(next(joint_values))
                joint_origins.append(transform[:3, 3].copy())
                joint_axes.append(transform[:3, :3] @ spec.axis)
                transform = transform @ _rotation_about_axis(spec.axis, joint_value)
            elif spec.joint_type == "fixed":
                continue
            else:
                raise ValueError(f"Unsupported joint type '{spec.joint_type}' in ABB mapper.")

        tcp_position = transform[:3, 3]
        jacobian = np.zeros((6, len(joint_axes)), dtype=np.float64)
        for index, (origin, axis_world) in enumerate(zip(joint_origins, joint_axes, strict=True)):
            jacobian[:3, index] = np.cross(axis_world, tcp_position - origin)
            jacobian[3:, index] = axis_world

        return transform, jacobian


def build_openpi_observation(
    request_payload: dict[str, Any],
    *,
    mapper: AbbLiberoMapper | None,
    state_dim: int,
    image_size: int,
    prompt: str,
) -> tuple[dict[str, Any], np.ndarray]:
    observation = request_payload.get("observation", {})
    current_positions = np.asarray(observation.get("joint_positions", []), dtype=np.float32)
    if current_positions.ndim != 1 or current_positions.size == 0:
        raise ValueError("observation.joint_positions must be a non-empty 1D vector.")

    if mapper is None:
        state = np.zeros(state_dim, dtype=np.float32)
        copy_count = min(state_dim, current_positions.size)
        state[:copy_count] = current_positions[:copy_count]
    else:
        state = mapper.state_from_joint_positions(current_positions, state_dim=state_dim)

    zeros_image = np.zeros((image_size, image_size, 3), dtype=np.uint8)
    openpi_observation = {
        "observation/state": state,
        "observation/image": zeros_image,
        "observation/wrist_image": zeros_image,
        "prompt": str(request_payload.get("prompt", prompt)),
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
        }

    def infer(self, request_payload: dict[str, Any]) -> dict[str, Any]:
        policy = self._get_policy()
        openpi_observation, current_positions = build_openpi_observation(
            request_payload,
            mapper=self.mapper,
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
    parsed = parser.parse_args(args=args)

    logging.basicConfig(level=logging.INFO, force=True)

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
