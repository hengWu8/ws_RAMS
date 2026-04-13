#!/usr/bin/env python3
"""Read-only web dashboard for the ABB + dual RealSense + remote pi0 workcell.

The dashboard intentionally does not start ROS, EGM, controller_manager, or any
robot command path. It only polls read-only status endpoints and local device
enumeration commands so it is safe to run while the robot is powered.
"""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import requests
import shlex
import subprocess
import threading
import time
from typing import Any
import urllib.parse
import xml.etree.ElementTree as ElementTree


DEFAULT_RWS_IP = "192.168.125.1"
DEFAULT_RWS_USER = "Default User"
DEFAULT_RWS_PASSWORD = "robotics"
DEFAULT_REMOTE_HOST = "100.70.7.8"
DEFAULT_HOLD_POLICY_URL = "http://100.70.7.8:8001/healthz"
DEFAULT_OFFICIAL_POLICY_URL = "http://100.70.7.8:8002/healthz"
DEFAULT_WORKCELL_URDF = str(
    Path(__file__).resolve().parents[1] / "src" / "workcell_description" / "urdf" / "workcell.urdf"
)
DEFAULT_URDF_BASE_LINK = "base_link"
DEFAULT_URDF_TIP_LINK = "tool0"
DEFAULT_WORKSPACE_RADIUS_M = 0.5
DEFAULT_FRONT_CAMERA_DEVICE = "/dev/video10"
DEFAULT_WRIST_CAMERA_DEVICE = "/dev/video4"

CONTROL_PROCESS_PATTERNS = (
    "ros2_control_node",
    "abb_pi0_bridge_node",
    "controller_manager",
    "move_group",
    "robot_state_publisher",
    "realsense2_camera_node",
)

_URDF_CHAIN_CACHE: dict[tuple[str, str, str], list[dict[str, Any]]] = {}
_URDF_SCENE_CACHE: dict[tuple[str, str], dict[str, Any]] = {}
_CAMERA_LOCKS: dict[str, threading.Lock] = {}
_CAMERA_LAST_GOOD_JPEG: dict[str, bytes] = {}
_REQUESTS_SESSIONS: dict[tuple[str, str | None], requests.Session] = {}


def make_dashboard_config(**overrides: Any) -> dict[str, Any]:
    config = {
        "rws_ip": DEFAULT_RWS_IP,
        "rws_port": "80",
        "rws_user": DEFAULT_RWS_USER,
        "rws_password": DEFAULT_RWS_PASSWORD,
        "rws_timeout_s": 1.5,
        "remote_host": DEFAULT_REMOTE_HOST,
        "hold_policy_health_url": DEFAULT_HOLD_POLICY_URL,
        "official_policy_health_url": DEFAULT_OFFICIAL_POLICY_URL,
        "front_camera_device": DEFAULT_FRONT_CAMERA_DEVICE,
        "wrist_camera_device": DEFAULT_WRIST_CAMERA_DEVICE,
        "camera_width": 424,
        "camera_height": 240,
        "camera_jpeg_quality": 80,
        "workcell_urdf": DEFAULT_WORKCELL_URDF,
        "urdf_base_link": DEFAULT_URDF_BASE_LINK,
        "urdf_tip_link": DEFAULT_URDF_TIP_LINK,
        "workspace_radius_m": DEFAULT_WORKSPACE_RADIUS_M,
    }
    config.update(overrides)
    return config


def run_command(command: list[str], *, timeout_s: float = 2.0) -> dict[str, Any]:
    started = time.time()
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
        return {
            "ok": result.returncode == 0,
            "returncode": result.returncode,
            "stdout": result.stdout,
            "stderr": result.stderr,
            "elapsed_ms": round((time.time() - started) * 1000.0, 1),
        }
    except FileNotFoundError as exc:
        return {
            "ok": False,
            "returncode": None,
            "stdout": "",
            "stderr": f"{command[0]} not found: {exc}",
            "elapsed_ms": round((time.time() - started) * 1000.0, 1),
        }
    except subprocess.TimeoutExpired as exc:
        return {
            "ok": False,
            "returncode": None,
            "stdout": exc.stdout or "",
            "stderr": f"timeout after {timeout_s:.1f}s",
            "elapsed_ms": round((time.time() - started) * 1000.0, 1),
        }


def curl_json(url: str, *, timeout_s: float = 2.0, digest_user: str | None = None, digest_password: str | None = None) -> dict[str, Any]:
    started = time.time()
    payload: Any = None
    try:
        parsed = requests.utils.urlparse(url)
        session_key = (f"{parsed.scheme}://{parsed.netloc}", digest_user)
        session = _REQUESTS_SESSIONS.get(session_key)
        if session is None:
            session = requests.Session()
            session.trust_env = False
            session.headers.update({"Connection": "keep-alive"})
            if digest_user is not None and digest_password is not None:
                session.auth = requests.auth.HTTPDigestAuth(digest_user, digest_password)
            _REQUESTS_SESSIONS[session_key] = session
        response = session.get(url, timeout=timeout_s)
        body = response.text
        response.raise_for_status()
        payload = response.json()
        return {
            "ok": True,
            "returncode": response.status_code,
            "stderr": "",
            "elapsed_ms": round((time.time() - started) * 1000.0, 1),
            "payload": payload,
        }
    except Exception as exc:
        body = ""
        try:
            body = locals().get("response").text  # type: ignore[arg-type]
        except Exception:
            pass
        error_text = str(exc)
        if body and "Too many sessions" in body:
            error_text = body.strip()
        return {
            "ok": False,
            "returncode": None,
            "stderr": error_text,
            "elapsed_ms": round((time.time() - started) * 1000.0, 1),
            "payload": payload,
        }


def read_rws_jointtarget(config: dict[str, Any]) -> dict[str, Any]:
    url = f"http://{config['rws_ip']}:{config['rws_port']}/rw/motionsystem/mechunits/ROB_1/jointtarget?json=1"
    result = curl_json(
        url,
        timeout_s=config["rws_timeout_s"],
        digest_user=config["rws_user"],
        digest_password=config["rws_password"],
    )
    state = _first_rws_state(result.get("payload"))
    if not state:
        return {**result, "degrees": None, "radians": None}

    try:
        degrees = [float(state[f"rax_{index}"]) for index in range(1, 7)]
    except (KeyError, TypeError, ValueError) as exc:
        return {**result, "ok": False, "stderr": f"failed to parse jointtarget: {exc}", "degrees": None, "radians": None}

    return {
        **result,
        "degrees": degrees,
        "radians": [math.radians(value) for value in degrees],
    }


def read_rws_cartesian(config: dict[str, Any]) -> dict[str, Any]:
    url = f"http://{config['rws_ip']}:{config['rws_port']}/rw/motionsystem/mechunits/ROB_1/robtarget?json=1"
    result = curl_json(
        url,
        timeout_s=config["rws_timeout_s"],
        digest_user=config["rws_user"],
        digest_password=config["rws_password"],
    )
    state = _first_rws_state(result.get("payload"))
    if not state:
        return {**result, "position_mm": None, "quaternion": None, "configuration": None}

    try:
        position_mm = {axis: float(state[axis]) for axis in ("x", "y", "z")}
        quaternion = {axis: float(state[axis]) for axis in ("q1", "q2", "q3", "q4")}
        configuration = {axis: state.get(axis) for axis in ("cf1", "cf4", "cf6", "cfx")}
    except (KeyError, TypeError, ValueError) as exc:
        return {
            **result,
            "ok": False,
            "stderr": f"failed to parse robtarget: {exc}",
            "position_mm": None,
            "quaternion": None,
            "configuration": None,
        }

    return {
        **result,
        "position_mm": position_mm,
        "quaternion": quaternion,
        "configuration": configuration,
    }


def _first_rws_state(payload: Any) -> dict[str, Any] | None:
    try:
        states = payload["_embedded"]["_state"]
        if states:
            return states[0]
    except (KeyError, TypeError):
        return None
    return None


def read_cameras(config: dict[str, Any]) -> dict[str, Any]:
    devices = sorted(str(path) for path in Path("/dev").glob("video*"))
    v4l2 = run_command(["v4l2-ctl", "--list-devices"], timeout_s=2.0)
    groups = parse_v4l2_devices(v4l2.get("stdout", ""))
    realsense_groups = [group for group in groups if "RealSense" in group["name"]]
    front_info = describe_video_device(config["front_camera_device"])
    wrist_info = describe_video_device(config["wrist_camera_device"])
    return {
        "video_devices": devices,
        "front_device": config["front_camera_device"],
        "front_device_info": front_info,
        "wrist_device": config["wrist_camera_device"],
        "wrist_device_info": wrist_info,
        "v4l2": {
            "ok": v4l2["ok"],
            "stderr": v4l2["stderr"],
            "elapsed_ms": v4l2["elapsed_ms"],
        },
        "groups": groups,
        "realsense_count": len(realsense_groups),
        "realsense_groups": realsense_groups,
    }


def parse_v4l2_devices(raw_text: str) -> list[dict[str, Any]]:
    groups: list[dict[str, Any]] = []
    current: dict[str, Any] | None = None
    for line in raw_text.splitlines():
        if not line.strip():
            continue
        if not line.startswith("\t"):
            current = {"name": line.rstrip(":"), "devices": []}
            groups.append(current)
        elif current is not None:
            current["devices"].append(line.strip())
    return groups


def describe_video_device(device: str) -> dict[str, Any]:
    sysfs_path = Path("/sys/class/video4linux") / Path(device).name
    info = {
        "device": device,
        "name": None,
        "usb_product": None,
        "usb_id": None,
        "usb_path": None,
    }
    try:
        info["name"] = (sysfs_path / "name").read_text(encoding="utf-8").strip()
    except Exception:
        pass
    try:
        resolved_device = (sysfs_path / "device").resolve()
        info["usb_path"] = str(resolved_device)
        usb_node = resolved_device
        while usb_node != usb_node.parent:
            product_file = usb_node / "product"
            id_vendor_file = usb_node / "idVendor"
            id_product_file = usb_node / "idProduct"
            if product_file.exists():
                info["usb_product"] = product_file.read_text(encoding="utf-8").strip()
            if id_vendor_file.exists() and id_product_file.exists():
                info["usb_id"] = f"{id_vendor_file.read_text(encoding='utf-8').strip()}:{id_product_file.read_text(encoding='utf-8').strip()}"
            if info["usb_product"] or info["usb_id"]:
                break
            usb_node = usb_node.parent
    except Exception:
        pass
    return info


def capture_camera_jpeg(device: str, *, width: int, height: int, quality: int = 80) -> tuple[bytes, str]:
    try:
        import cv2
        import numpy as np  # noqa: F401
    except Exception as exc:  # pragma: no cover - depends on local OpenCV install
        return _camera_error_jpeg(f"OpenCV unavailable: {exc}"), "image/jpeg"

    lock = _CAMERA_LOCKS.setdefault(device, threading.Lock())
    with lock:
        last_good = _CAMERA_LAST_GOOD_JPEG.get(device)
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        try:
            if not capture.isOpened():
                if last_good is not None:
                    return last_good, "image/jpeg"
                return _camera_error_jpeg(f"{device} not open"), "image/jpeg"
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            capture.set(cv2.CAP_PROP_FPS, 15)

            frame = None
            ok = False
            for _ in range(8):
                ok, frame = capture.read()
                if ok and frame is not None:
                    break
                time.sleep(0.03)
            if not ok or frame is None:
                if last_good is not None:
                    return last_good, "image/jpeg"
                return _camera_error_jpeg(f"{device} no frame"), "image/jpeg"

            encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
            encoded_ok, encoded = cv2.imencode(".jpg", frame, encode_params)
            if not encoded_ok:
                if last_good is not None:
                    return last_good, "image/jpeg"
                return _camera_error_jpeg(f"{device} JPEG encode failed"), "image/jpeg"
            payload = encoded.tobytes()
            _CAMERA_LAST_GOOD_JPEG[device] = payload
            return payload, "image/jpeg"
        finally:
            capture.release()


def _camera_error_jpeg(message: str) -> bytes:
    import cv2
    import numpy as np

    image = np.full((240, 424, 3), (42, 47, 50), dtype=np.uint8)
    cv2.putText(image, "camera unavailable", (18, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (235, 238, 238), 2)
    cv2.putText(image, message[:48], (18, 136), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (190, 205, 210), 1)
    ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    return encoded.tobytes() if ok else b""


def read_urdf_robot_model(config: dict[str, Any], joint_positions: list[float] | None) -> dict[str, Any]:
    if joint_positions is None:
        return {"ok": False, "error": "joint positions unavailable", "points": []}
    joint_values = {f"joint_{index}": value for index, value in enumerate(joint_positions, start=1)}
    try:
        chain = _load_urdf_chain(
            config["workcell_urdf"],
            config["urdf_base_link"],
            config["urdf_tip_link"],
        )
        points = _compute_chain_points(chain, joint_values)
        return {
            "ok": True,
            "urdf": config["workcell_urdf"],
            "base_link": config["urdf_base_link"],
            "tip_link": config["urdf_tip_link"],
            "points": points,
        }
    except Exception as exc:
        return {
            "ok": False,
            "error": str(exc),
            "urdf": config["workcell_urdf"],
            "base_link": config["urdf_base_link"],
            "tip_link": config["urdf_tip_link"],
            "points": [],
        }


def read_urdf_scene_model(config: dict[str, Any], joint_positions: list[float] | None) -> dict[str, Any]:
    if joint_positions is None:
        return {"ok": False, "error": "joint positions unavailable", "visuals": [], "link_frames": {}, "tip_frame": None}
    joint_values = {f"joint_{index}": value for index, value in enumerate(joint_positions, start=1)}
    try:
        scene = _load_urdf_scene(config["workcell_urdf"], config["urdf_base_link"])
        resolved_joint_values = _resolve_joint_values(scene["joints"], joint_values)
        visuals, link_frames = _compute_scene_visuals(scene, config["urdf_base_link"], resolved_joint_values)
        tip_frame = link_frames.get(config["urdf_tip_link"])
        return {
            "ok": True,
            "urdf": config["workcell_urdf"],
            "base_link": config["urdf_base_link"],
            "tip_link": config["urdf_tip_link"],
            "visuals": visuals,
            "link_frames": link_frames,
            "tip_frame": tip_frame,
        }
    except Exception as exc:
        return {
            "ok": False,
            "error": str(exc),
            "urdf": config["workcell_urdf"],
            "base_link": config["urdf_base_link"],
            "tip_link": config["urdf_tip_link"],
            "visuals": [],
            "link_frames": {},
            "tip_frame": None,
        }


def _load_urdf_chain(urdf_path: str, base_link: str, tip_link: str) -> list[dict[str, Any]]:
    cache_key = (str(Path(urdf_path).resolve()), base_link, tip_link)
    if cache_key in _URDF_CHAIN_CACHE:
        return _URDF_CHAIN_CACHE[cache_key]

    root = ElementTree.parse(urdf_path).getroot()
    child_to_joint: dict[str, dict[str, Any]] = {}
    for joint_element in root.findall("joint"):
        origin = joint_element.find("origin")
        xyz = _parse_vector(origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0")
        rpy = _parse_vector(origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0")
        axis_element = joint_element.find("axis")
        axis = _parse_vector(axis_element.attrib.get("xyz", "1 0 0") if axis_element is not None else "1 0 0")
        parent = joint_element.find("parent").attrib["link"]
        child = joint_element.find("child").attrib["link"]
        child_to_joint[child] = {
            "name": joint_element.attrib["name"],
            "type": joint_element.attrib.get("type", "fixed"),
            "parent": parent,
            "child": child,
            "xyz": xyz,
            "rpy": rpy,
            "axis": _normalize(axis),
        }

    chain: list[dict[str, Any]] = []
    current = tip_link
    while current != base_link:
        if current not in child_to_joint:
            raise ValueError(f"no URDF chain from {base_link!r} to {tip_link!r}; stopped at {current!r}")
        joint = child_to_joint[current]
        chain.append(joint)
        current = joint["parent"]
    chain.reverse()
    _URDF_CHAIN_CACHE[cache_key] = chain
    return chain


def _load_urdf_scene(urdf_path: str, base_link: str) -> dict[str, Any]:
    cache_key = (str(Path(urdf_path).resolve()), base_link)
    if cache_key in _URDF_SCENE_CACHE:
        return _URDF_SCENE_CACHE[cache_key]

    root = ElementTree.parse(urdf_path).getroot()
    links: dict[str, dict[str, Any]] = {}
    for link_element in root.findall("link"):
        link_name = link_element.attrib["name"]
        visuals = []
        for visual_element in link_element.findall("visual"):
            origin = visual_element.find("origin")
            xyz = _parse_vector(origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0")
            rpy = _parse_vector(origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0")
            geometry_element = visual_element.find("geometry")
            if geometry_element is None:
                continue
            geometry: dict[str, Any] | None = None
            mesh_element = geometry_element.find("mesh")
            if mesh_element is not None:
                geometry = {
                    "type": "mesh",
                    "path": _resolve_package_path(mesh_element.attrib["filename"], urdf_path),
                    "scale": [float(v) for v in mesh_element.attrib.get("scale", "1 1 1").split()],
                }
            box_element = geometry_element.find("box")
            if box_element is not None:
                geometry = {
                    "type": "box",
                    "size": [float(v) for v in box_element.attrib["size"].split()],
                }
            cylinder_element = geometry_element.find("cylinder")
            if cylinder_element is not None:
                geometry = {
                    "type": "cylinder",
                    "radius": float(cylinder_element.attrib["radius"]),
                    "length": float(cylinder_element.attrib["length"]),
                }
            sphere_element = geometry_element.find("sphere")
            if sphere_element is not None:
                geometry = {
                    "type": "sphere",
                    "radius": float(sphere_element.attrib["radius"]),
                }
            if geometry is None:
                continue
            visuals.append(
                {
                    "xyz": [float(v) for v in xyz],
                    "rpy": [float(v) for v in rpy],
                    "geometry": geometry,
                }
            )
        links[link_name] = {"name": link_name, "visuals": visuals}

    joints: dict[str, dict[str, Any]] = {}
    children_by_parent: dict[str, list[str]] = {}
    for joint_element in root.findall("joint"):
        origin = joint_element.find("origin")
        xyz = _parse_vector(origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0")
        rpy = _parse_vector(origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0")
        axis_element = joint_element.find("axis")
        axis = _parse_vector(axis_element.attrib.get("xyz", "1 0 0") if axis_element is not None else "1 0 0")
        parent = joint_element.find("parent").attrib["link"]
        child = joint_element.find("child").attrib["link"]
        mimic_element = joint_element.find("mimic")
        mimic = None
        if mimic_element is not None:
            mimic = {
                "joint": mimic_element.attrib["joint"],
                "multiplier": float(mimic_element.attrib.get("multiplier", "1.0")),
                "offset": float(mimic_element.attrib.get("offset", "0.0")),
            }
        joint = {
            "name": joint_element.attrib["name"],
            "type": joint_element.attrib.get("type", "fixed"),
            "parent": parent,
            "child": child,
            "xyz": xyz,
            "rpy": rpy,
            "axis": _normalize(axis),
            "mimic": mimic,
        }
        joints[joint["name"]] = joint
        children_by_parent.setdefault(parent, []).append(joint["name"])

    scene = {
        "base_link": base_link,
        "links": links,
        "joints": joints,
        "children_by_parent": children_by_parent,
    }
    _URDF_SCENE_CACHE[cache_key] = scene
    return scene


def _compute_chain_points(chain: list[dict[str, Any]], joint_values: dict[str, float]) -> list[dict[str, Any]]:
    import numpy as np

    transform = np.eye(4, dtype=float)
    points = [
        {
            "link": chain[0]["parent"] if chain else "base_link",
            "joint": None,
            "type": "base",
            "position_m": [0.0, 0.0, 0.0],
        }
    ]
    for joint in chain:
        transform = transform @ _transform_from_xyz_rpy(joint["xyz"], joint["rpy"])
        if joint["type"] in {"revolute", "continuous"}:
            transform = transform @ _rotation_about_axis(joint["axis"], float(joint_values.get(joint["name"], 0.0)))
        elif joint["type"] == "prismatic":
            transform = transform @ _translation_along_axis(joint["axis"], float(joint_values.get(joint["name"], 0.0)))
        points.append(
            {
                "link": joint["child"],
                "joint": joint["name"],
                "type": joint["type"],
                "position_m": [float(v) for v in transform[:3, 3]],
            }
        )
    return points


def _resolve_joint_values(joints: dict[str, dict[str, Any]], seed_joint_values: dict[str, float]) -> dict[str, float]:
    resolved = dict(seed_joint_values)
    for _ in range(len(joints) + 2):
        updated = False
        for joint_name, joint in joints.items():
            if joint_name in resolved:
                continue
            mimic = joint.get("mimic")
            if mimic is not None:
                source_name = mimic["joint"]
                if source_name not in resolved:
                    continue
                resolved[joint_name] = float(resolved[source_name]) * float(mimic["multiplier"]) + float(mimic["offset"])
                updated = True
                continue
            resolved[joint_name] = 0.0
            updated = True
        if not updated:
            break
    return resolved


def _compute_scene_visuals(scene: dict[str, Any], base_link: str, joint_values: dict[str, float]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    import numpy as np

    link_transforms: dict[str, Any] = {base_link: np.eye(4, dtype=float)}
    ordered_links = [base_link]
    queue = [base_link]
    while queue:
        parent_link = queue.pop(0)
        parent_transform = link_transforms[parent_link]
        for joint_name in scene["children_by_parent"].get(parent_link, []):
            joint = scene["joints"][joint_name]
            child_transform = parent_transform @ _transform_from_xyz_rpy(joint["xyz"], joint["rpy"]) @ _joint_motion_transform(
                joint,
                joint_values,
            )
            link_transforms[joint["child"]] = child_transform
            ordered_links.append(joint["child"])
            queue.append(joint["child"])

    visuals = []
    for link_name in ordered_links:
        link = scene["links"].get(link_name)
        if not link:
            continue
        link_transform = link_transforms[link_name]
        for visual_index, visual in enumerate(link.get("visuals", [])):
            visual_transform = link_transform @ _transform_from_xyz_rpy(visual["xyz"], visual["rpy"])
            visuals.append(
                {
                    "link": link_name,
                    "visual_index": visual_index,
                    "geometry": visual["geometry"],
                    "transform": [[float(v) for v in row] for row in visual_transform],
                }
            )
    serialized_frames = {
        link_name: [[float(v) for v in row] for row in transform]
        for link_name, transform in link_transforms.items()
    }
    return visuals, serialized_frames


def _parse_vector(raw: str):
    import numpy as np

    values = [float(part) for part in raw.split()]
    if len(values) != 3:
        raise ValueError(f"expected 3-vector, got {raw!r}")
    return np.asarray(values, dtype=float)


def _normalize(vector):
    import numpy as np

    norm = float(np.linalg.norm(vector))
    return vector / norm if norm > 1e-12 else vector


def _rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float):
    import numpy as np

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _transform_from_xyz_rpy(xyz, rpy):
    import numpy as np

    transform = np.eye(4, dtype=float)
    transform[:3, :3] = _rotation_matrix_from_rpy(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    transform[:3, 3] = xyz
    return transform


def _rotation_about_axis(axis, angle: float):
    import numpy as np

    axis = _normalize(axis)
    x, y, z = axis
    skew = np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float)
    rotation = np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    return transform


def _translation_along_axis(axis, distance: float):
    import numpy as np

    transform = np.eye(4, dtype=float)
    transform[:3, 3] = _normalize(axis) * distance
    return transform


def _joint_motion_transform(joint: dict[str, Any], joint_values: dict[str, float]):
    joint_type = joint["type"]
    value = float(joint_values.get(joint["name"], 0.0))
    if joint_type in {"revolute", "continuous"}:
        return _rotation_about_axis(joint["axis"], value)
    if joint_type == "prismatic":
        return _translation_along_axis(joint["axis"], value)
    import numpy as np

    return np.eye(4, dtype=float)


def _resolve_package_path(raw_path: str, urdf_path: str) -> str:
    if raw_path.startswith("package://"):
        package_spec = raw_path.removeprefix("package://")
        package_name, relative = package_spec.split("/", 1)
        repo_candidate = Path(__file__).resolve().parents[1] / "src" / package_name / relative
        if repo_candidate.exists():
            return str(repo_candidate)
        urdf_resolved = Path(urdf_path).resolve()
        for parent in urdf_resolved.parents:
            candidate = parent / package_name / relative
            if candidate.exists():
                return str(candidate)
    candidate = Path(raw_path)
    if candidate.is_absolute():
        return str(candidate)
    return str((Path(urdf_path).resolve().parent / candidate).resolve())


def read_policy_health(config: dict[str, Any]) -> dict[str, Any]:
    return {
        "hold": curl_json(config["hold_policy_health_url"], timeout_s=2.0),
        "official": curl_json(config["official_policy_health_url"], timeout_s=2.0),
    }


def read_tailscale(config: dict[str, Any]) -> dict[str, Any]:
    status = run_command(["tailscale", "status"], timeout_s=2.0)
    remote_lines = [
        line
        for line in status.get("stdout", "").splitlines()
        if config["remote_host"] in line or "tjzs-desktop" in line
    ]
    return {
        "ok": status["ok"],
        "remote_host": config["remote_host"],
        "remote_lines": remote_lines,
        "stderr": status["stderr"],
        "elapsed_ms": status["elapsed_ms"],
    }


def read_control_processes() -> dict[str, Any]:
    processes = []
    current_pid = os.getpid()
    for pattern in CONTROL_PROCESS_PATTERNS:
        result = run_command(["pgrep", "-af", pattern], timeout_s=1.0)
        for line in result.get("stdout", "").splitlines():
            parts = line.split(maxsplit=1)
            if not parts:
                continue
            try:
                pid = int(parts[0])
            except ValueError:
                continue
            command = parts[1] if len(parts) > 1 else ""
            if pid == current_pid or "workcell_dashboard.py" in command:
                continue
            processes.append({"pattern": pattern, "pid": pid, "command": command})
    return {
        "ok": len(processes) == 0,
        "processes": processes,
        "patterns": list(CONTROL_PROCESS_PATTERNS),
    }


def build_status(config: dict[str, Any]) -> dict[str, Any]:
    timestamp = time.time()
    robot_joints = read_rws_jointtarget(config)
    robot_cartesian = read_rws_cartesian(config)
    robot_urdf = read_urdf_robot_model(config, robot_joints.get("radians"))
    robot_scene = read_urdf_scene_model(config, robot_joints.get("radians"))
    return {
        "timestamp_sec": timestamp,
        "timestamp_text": time.strftime("%Y-%m-%d %H:%M:%S %Z", time.localtime(timestamp)),
        "config": {
            "rws_ip": config["rws_ip"],
            "rws_port": config["rws_port"],
            "remote_host": config["remote_host"],
            "hold_policy_health_url": config["hold_policy_health_url"],
            "official_policy_health_url": config["official_policy_health_url"],
            "front_camera_device": config["front_camera_device"],
            "wrist_camera_device": config["wrist_camera_device"],
            "workcell_urdf": config["workcell_urdf"],
            "workspace_radius_m": config["workspace_radius_m"],
        },
        "robot": {
            "jointtarget": robot_joints,
            "cartesian": robot_cartesian,
            "urdf_model": robot_urdf,
            "scene_model": robot_scene,
        },
        "cameras": read_cameras(config),
        "policy": read_policy_health(config),
        "tailscale": read_tailscale(config),
        "control_processes": read_control_processes(),
        "system": {
            "loadavg": os.getloadavg() if hasattr(os, "getloadavg") else None,
        },
    }


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "WorkcellDashboard/0.1"

    def do_GET(self) -> None:
        parsed_path = urllib.parse.urlparse(self.path)
        if parsed_path.path in ("/", "/index.html"):
            self._write(200, HTML.encode("utf-8"), "text/html; charset=utf-8")
            return
        if parsed_path.path == "/api/status":
            payload = build_status(getattr(self.server, "dashboard_config"))
            self._write_json(200, payload)
            return
        if parsed_path.path.startswith("/api/camera/") and parsed_path.path.endswith(".jpg"):
            camera_name = parsed_path.path.removeprefix("/api/camera/").removesuffix(".jpg")
            config = getattr(self.server, "dashboard_config")
            device_by_name = {
                "front": config["front_camera_device"],
                "wrist": config["wrist_camera_device"],
            }
            if camera_name not in device_by_name:
                self._write_json(404, {"error": "unknown_camera", "camera": camera_name})
                return
            body, content_type = capture_camera_jpeg(
                device_by_name[camera_name],
                width=config["camera_width"],
                height=config["camera_height"],
                quality=config["camera_jpeg_quality"],
            )
            self._write(200, body, content_type)
            return
        self._write_json(404, {"error": "not_found"})

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"{self.address_string()} - {fmt % args}")

    def _write_json(self, status_code: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self._write(status_code, body, "application/json; charset=utf-8")

    def _write(self, status_code: int, body: bytes, content_type: str) -> None:
        self.send_response(status_code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>ws_RAMS Workcell Dashboard</title>
  <style>
    :root {
      --ink: #16201d;
      --muted: #65736f;
      --paper: #f4efe4;
      --card: rgba(255, 252, 244, 0.86);
      --line: rgba(22, 32, 29, 0.14);
      --good: #147d64;
      --warn: #b36b00;
      --bad: #b43131;
      --cyan: #0f7892;
      --amber: #e0a929;
      --shadow: 0 22px 70px rgba(29, 25, 18, 0.14);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      color: var(--ink);
      font-family: "Aptos Display", "Avenir Next", "Segoe UI Variable Display", sans-serif;
      background:
        radial-gradient(circle at 8% 8%, rgba(15, 120, 146, 0.20), transparent 32rem),
        radial-gradient(circle at 88% 0%, rgba(224, 169, 41, 0.28), transparent 26rem),
        linear-gradient(135deg, #f7f1e5 0%, #e7ded0 55%, #d7e1dd 100%);
      min-height: 100vh;
    }
    .shell { width: min(1500px, calc(100vw - 32px)); margin: 0 auto; padding: 28px 0 42px; }
    .hero {
      display: grid;
      grid-template-columns: 1.4fr 0.8fr;
      gap: 18px;
      align-items: stretch;
      margin-bottom: 18px;
    }
    .title, .card {
      background: var(--card);
      border: 1px solid var(--line);
      border-radius: 28px;
      box-shadow: var(--shadow);
      backdrop-filter: blur(18px);
    }
    .title { padding: 30px; position: relative; overflow: hidden; }
    .title:after {
      content: "";
      position: absolute;
      right: -70px;
      top: -95px;
      width: 260px;
      height: 260px;
      border-radius: 999px;
      border: 38px solid rgba(15, 120, 146, 0.11);
    }
    h1 { margin: 0 0 10px; font-size: clamp(34px, 4.5vw, 68px); letter-spacing: -0.06em; line-height: 0.94; }
    .subtitle { max-width: 760px; color: var(--muted); font-size: 17px; line-height: 1.55; }
    .pill-row { display: flex; flex-wrap: wrap; gap: 10px; margin-top: 22px; }
    .pill {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      border: 1px solid var(--line);
      border-radius: 999px;
      padding: 8px 12px;
      background: rgba(255,255,255,0.5);
      font-weight: 720;
      color: var(--ink);
    }
    .dot { width: 9px; height: 9px; border-radius: 999px; background: var(--muted); box-shadow: 0 0 0 4px rgba(101,115,111,0.12); }
    .dot.good { background: var(--good); box-shadow: 0 0 0 4px rgba(20,125,100,0.14); }
    .dot.warn { background: var(--warn); box-shadow: 0 0 0 4px rgba(179,107,0,0.14); }
    .dot.bad { background: var(--bad); box-shadow: 0 0 0 4px rgba(180,49,49,0.14); }
    .card { padding: 20px; }
    .status-card { display: grid; gap: 13px; align-content: center; }
    .metric { display: flex; justify-content: space-between; gap: 16px; padding: 10px 0; border-bottom: 1px solid var(--line); }
    .metric:last-child { border-bottom: 0; }
    .label { color: var(--muted); font-size: 13px; text-transform: uppercase; letter-spacing: 0.10em; }
    .value { font-weight: 780; text-align: right; }
    .grid { display: grid; grid-template-columns: 1.15fr 0.85fr; gap: 18px; }
    .stack { display: grid; gap: 18px; }
    .section-title { display: flex; align-items: center; justify-content: space-between; gap: 12px; margin-bottom: 14px; }
    h2 { margin: 0; font-size: 22px; letter-spacing: -0.03em; }
    .small { color: var(--muted); font-size: 13px; }
    .pose-layout { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; align-items: center; }
    .robot-svg { width: 100%; min-height: 420px; background: linear-gradient(180deg, rgba(255,255,255,0.62), rgba(255,255,255,0.24)); border: 1px solid var(--line); border-radius: 24px; }
    .joint-list { display: grid; gap: 10px; }
    .joint { display: grid; grid-template-columns: 58px 1fr 86px; gap: 10px; align-items: center; }
    .bar { height: 12px; border-radius: 999px; background: rgba(22,32,29,0.10); overflow: hidden; }
    .bar > span { display: block; height: 100%; width: 50%; border-radius: inherit; background: linear-gradient(90deg, var(--cyan), var(--amber)); }
    .mono { font-family: "Cascadia Code", "SFMono-Regular", "Roboto Mono", monospace; font-size: 13px; }
    .camera-grid { display: grid; gap: 12px; }
    .preview-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 12px; margin-bottom: 14px; }
    .preview-card { border: 1px solid var(--line); border-radius: 20px; overflow: hidden; background: rgba(22,32,29,0.08); }
    .preview-card img { width: 100%; display: block; aspect-ratio: 16 / 9; object-fit: cover; background: #20272a; }
    .preview-caption { display: flex; justify-content: space-between; gap: 10px; padding: 10px 12px; font-weight: 800; }
    .camera-card { border: 1px solid var(--line); border-radius: 20px; padding: 14px; background: rgba(255,255,255,0.46); }
    .camera-name { font-weight: 800; margin-bottom: 8px; }
    .devs { display: flex; flex-wrap: wrap; gap: 7px; }
    .dev { padding: 5px 8px; border-radius: 9px; background: rgba(15,120,146,0.10); color: #0c5e72; font-size: 12px; }
    pre {
      margin: 0;
      padding: 14px;
      max-height: 220px;
      overflow: auto;
      border-radius: 18px;
      background: rgba(22,32,29,0.08);
      border: 1px solid var(--line);
      white-space: pre-wrap;
    }
    .error { color: var(--bad); }
    .warn-text { color: var(--warn); }
    .ok-text { color: var(--good); }
    @media (max-width: 980px) {
      .hero, .grid, .pose-layout, .preview-grid { grid-template-columns: 1fr; }
      .shell { width: min(100vw - 20px, 900px); padding-top: 10px; }
    }
  </style>
</head>
<body>
  <main class="shell">
    <section class="hero">
      <div class="title">
        <h1>ws_RAMS<br />Workcell Board</h1>
        <div class="subtitle">Read-only view of ABB RWS pose, dual RealSense presence, remote pi0 policy adapters, and local ROS control safety state. No command path is started by this dashboard.</div>
        <div class="pill-row">
          <span class="pill"><span id="robot-dot" class="dot"></span><span id="robot-pill">Robot RWS</span></span>
          <span class="pill"><span id="camera-dot" class="dot"></span><span id="camera-pill">Cameras</span></span>
          <span class="pill"><span id="policy-dot" class="dot"></span><span id="policy-pill">Remote policy</span></span>
          <span class="pill"><span id="safety-dot" class="dot"></span><span id="safety-pill">Control chain</span></span>
        </div>
      </div>
      <div class="card status-card">
        <div class="metric"><span class="label">Updated</span><span id="updated" class="value">-</span></div>
        <div class="metric"><span class="label">RWS</span><span id="rws-target" class="value">-</span></div>
        <div class="metric"><span class="label">Remote</span><span id="remote-target" class="value">-</span></div>
        <div class="metric"><span class="label">Load</span><span id="loadavg" class="value">-</span></div>
      </div>
    </section>
    <section class="grid">
      <div class="stack">
        <div class="card">
          <div class="section-title">
            <h2>Robot Pose</h2>
            <span id="cartesian-summary" class="small">Waiting for RWS...</span>
          </div>
          <div class="pose-layout">
            <svg id="robot-svg" class="robot-svg" viewBox="0 0 520 340" role="img" aria-label="Robot joint pose visualization"></svg>
            <div id="joint-list" class="joint-list"></div>
          </div>
        </div>
        <div class="card">
          <div class="section-title">
            <h2>Camera Stack</h2>
            <span id="camera-summary" class="small">-</span>
          </div>
          <div class="preview-grid">
            <div class="preview-card">
              <img id="front-feed" alt="front RealSense live preview" />
              <div class="preview-caption"><span>front RGB</span><span id="front-device" class="small">-</span></div>
            </div>
            <div class="preview-card">
              <img id="wrist-feed" alt="wrist RealSense live preview" />
              <div class="preview-caption"><span>wrist RGB</span><span id="wrist-device" class="small">-</span></div>
            </div>
          </div>
          <div id="camera-grid" class="camera-grid"></div>
        </div>
      </div>
      <div class="stack">
        <div class="card">
          <div class="section-title">
            <h2>Policy Links</h2>
            <span class="small">Tailscale + HTTP health</span>
          </div>
          <div id="policy-list"></div>
        </div>
        <div class="card">
          <div class="section-title">
            <h2>Safety Watch</h2>
            <span class="small">Local process detection</span>
          </div>
          <div id="safety-list"></div>
        </div>
        <div class="card">
          <div class="section-title">
            <h2>Raw Snapshot</h2>
            <span class="small">Last API payload</span>
          </div>
          <pre id="raw">Loading...</pre>
        </div>
      </div>
    </section>
  </main>
  <script>
    const $ = (id) => document.getElementById(id);
    const fmt = (n, d = 3) => Number.isFinite(n) ? n.toFixed(d) : "-";
    function dot(id, state) {
      const el = $(id);
      el.className = "dot " + state;
    }
    function healthClass(ok) { return ok ? "good" : "bad"; }
    function setText(id, text) { $(id).textContent = text; }
    function renderRobot(model, deg) {
      const svg = $("robot-svg");
      if (!model || !model.ok || !model.points || model.points.length < 2) {
        const fallback = model && model.error ? model.error : "No URDF FK data";
        svg.innerHTML = `<text x="260" y="170" text-anchor="middle" fill="#65736f">${fallback}</text>`;
        return;
      }
      const raw = model.points.map(p => ({...p, v: p.position_m}));
      const xs = raw.map(p => p.v[0]);
      const ys = raw.map(p => p.v[1]);
      const zs = raw.map(p => p.v[2]);
      const minX = Math.min(...xs), maxX = Math.max(...xs);
      const minY = Math.min(...ys), maxY = Math.max(...ys);
      const minZ = Math.min(...zs), maxZ = Math.max(...zs);
      const padRange = (a, b) => Math.max(Math.abs(b - a), 0.35);
      const rangeFront = Math.max(padRange(minX, maxX), padRange(minZ, maxZ));
      const rangeTop = Math.max(padRange(minX, maxX), padRange(minY, maxY));
      const projectFront = (p) => ({
        x: 40 + ((p.v[0] - (minX + maxX) / 2) / rangeFront + 0.5) * 200,
        y: 302 - ((p.v[2] - (minZ + maxZ) / 2) / rangeFront + 0.5) * 250,
      });
      const projectTop = (p) => ({
        x: 292 + ((p.v[0] - (minX + maxX) / 2) / rangeTop + 0.5) * 190,
        y: 302 - ((p.v[1] - (minY + maxY) / 2) / rangeTop + 0.5) * 250,
      });
      const drawView = (project, title, x0) => {
        const pts = raw.map(project);
        const segments = pts.slice(1).map((p, i) => {
          const a = pts[i];
          const joint = raw[i + 1].joint || raw[i + 1].link;
          const strong = /^joint_[1-6]$/.test(String(joint));
          return `<line x1="${a.x}" y1="${a.y}" x2="${p.x}" y2="${p.y}" stroke="${strong ? "#0f7892" : "rgba(22,32,29,0.34)"}" stroke-width="${strong ? 8 : 4}" stroke-linecap="round"/>`;
        }).join("");
        const nodes = pts.map((p, i) => {
          const label = raw[i].joint && /^joint_[1-6]$/.test(raw[i].joint) ? raw[i].joint.replace("joint_", "J") : "";
          return `<g><circle cx="${p.x}" cy="${p.y}" r="${label ? 10 : 6}" fill="${label ? "#fff8e8" : "#16201d"}" stroke="rgba(22,32,29,0.20)"/>${label ? `<text x="${p.x}" y="${p.y + 4}" text-anchor="middle" font-size="9" font-weight="900">${label}</text>` : ""}</g>`;
        }).join("");
        return `
          <text x="${x0}" y="32" fill="#65736f" font-size="13" font-weight="800">${title}</text>
          <rect x="${x0}" y="48" width="210" height="270" rx="20" fill="rgba(255,255,255,0.48)" stroke="rgba(22,32,29,0.12)"/>
          ${segments}${nodes}
        `;
      };
      svg.innerHTML = `
        <defs>
          <filter id="glow"><feGaussianBlur stdDeviation="3" result="b"/><feMerge><feMergeNode in="b"/><feMergeNode in="SourceGraphic"/></feMerge></filter>
        </defs>
        ${drawView(projectFront, `URDF front X/Z: ${model.base_link} -> ${model.tip_link}`, 24)}
        ${drawView(projectTop, "URDF top X/Y", 276)}
        <text x="28" y="334" fill="#65736f" font-size="12">FK from workcell URDF, driven by live RWS jointtarget.</text>
      `;
    }
    function renderJoints(deg, rad) {
      const list = $("joint-list");
      if (!deg) {
        list.innerHTML = `<div class="error">RWS jointtarget unavailable.</div>`;
        return;
      }
      list.innerHTML = deg.map((value, i) => {
        const pct = Math.max(0, Math.min(100, 50 + value / 180 * 50));
        return `<div class="joint">
          <strong>J${i + 1}</strong>
          <div class="bar"><span style="width:${pct}%"></span></div>
          <span class="mono">${fmt(value, 2)} deg</span>
          <span></span><span class="small mono">${fmt(rad[i], 6)} rad</span><span></span>
        </div>`;
      }).join("");
    }
    function renderCameras(cameras) {
      const groups = cameras.groups || [];
      setText("camera-summary", `${cameras.realsense_count || 0} RealSense groups, ${(cameras.video_devices || []).length} /dev/video nodes`);
      setText("front-device", cameras.front_device || "-");
      setText("wrist-device", cameras.wrist_device || "-");
      $("camera-grid").innerHTML = groups.map(group => `
        <div class="camera-card">
          <div class="camera-name">${group.name}</div>
          <div class="devs">${(group.devices || []).map(dev => `<span class="dev">${dev}</span>`).join("")}</div>
        </div>
      `).join("") || `<div class="warn-text">No v4l2 camera groups found.</div>`;
    }
    function renderPolicies(policy, tailscale) {
      const hold = policy.hold || {};
      const official = policy.official || {};
      const row = (name, item) => {
        const payload = item.payload || {};
        const ok = Boolean(item.ok && (payload.ok !== false));
        return `<div class="metric"><span class="label">${name}</span><span class="value ${ok ? "ok-text" : "error"}">${ok ? "OK" : "DOWN"} ${payload.policy_loader ? "(" + payload.policy_loader + ")" : ""}</span></div>`;
      };
      $("policy-list").innerHTML = `
        ${row("hold adapter", hold)}
        ${row("official adapter", official)}
        <div class="metric"><span class="label">tailscale</span><span class="value">${(tailscale.remote_lines || []).join(" | ") || "not found"}</span></div>
      `;
    }
    function renderSafety(control) {
      const procs = control.processes || [];
      if (!procs.length) {
        $("safety-list").innerHTML = `<div class="ok-text">No local ROS/ABB control processes detected by dashboard watchlist.</div>`;
        return;
      }
      $("safety-list").innerHTML = `<div class="error">Detected control-related processes:</div><pre>${procs.map(p => `${p.pid} [${p.pattern}] ${p.command}`).join("\n")}</pre>`;
    }
    async function refresh() {
      try {
        const response = await fetch("/api/status", {cache: "no-store"});
        const data = await response.json();
        const joint = data.robot.jointtarget;
        const cart = data.robot.cartesian;
        const deg = joint.degrees;
        const rad = joint.radians;
        setText("updated", data.timestamp_text);
        setText("rws-target", `${data.config.rws_ip}:${data.config.rws_port}`);
        setText("remote-target", data.config.remote_host);
        setText("loadavg", data.system.loadavg ? data.system.loadavg.map(v => fmt(v, 2)).join(" / ") : "-");
        if (cart.position_mm) {
          const p = cart.position_mm;
          setText("cartesian-summary", `TCP x=${fmt(p.x, 1)} y=${fmt(p.y, 1)} z=${fmt(p.z, 1)} mm`);
        } else {
          setText("cartesian-summary", "TCP unavailable");
        }
        renderRobot(data.robot.urdf_model, deg);
        renderJoints(deg, rad || []);
        renderCameras(data.cameras);
        renderPolicies(data.policy, data.tailscale);
        renderSafety(data.control_processes);
        dot("robot-dot", joint.ok && cart.ok ? "good" : "bad");
        setText("robot-pill", joint.ok && cart.ok ? "Robot RWS OK" : "Robot RWS issue");
        dot("camera-dot", (data.cameras.realsense_count || 0) >= 2 ? "good" : "warn");
        setText("camera-pill", `${data.cameras.realsense_count || 0} RealSense`);
        const policyOk = Boolean(data.policy.hold?.ok || data.policy.official?.ok);
        dot("policy-dot", policyOk ? "good" : "bad");
        setText("policy-pill", policyOk ? "Policy reachable" : "Policy down");
        dot("safety-dot", data.control_processes.ok ? "good" : "warn");
        setText("safety-pill", data.control_processes.ok ? "No control procs" : "Control procs detected");
        $("raw").textContent = JSON.stringify(data, null, 2);
      } catch (error) {
        $("raw").textContent = String(error);
        dot("robot-dot", "bad");
        dot("policy-dot", "bad");
      }
    }
    function refreshCameraImages() {
      const stamp = Date.now();
      $("front-feed").src = `/api/camera/front.jpg?t=${stamp}`;
      $("wrist-feed").src = `/api/camera/wrist.jpg?t=${stamp}`;
    }
    refresh();
    refreshCameraImages();
    setInterval(refresh, 2000);
    setInterval(refreshCameraImages, 650);
  </script>
</body>
</html>
"""


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Read-only web dashboard for ws_RAMS ABB/pi0 workcell.")
    parser.add_argument("--host", default="127.0.0.1", help="Bind address. Use 0.0.0.0 for LAN access.")
    parser.add_argument("--port", type=int, default=8090, help="HTTP port.")
    parser.add_argument("--rws-ip", default=DEFAULT_RWS_IP, help="ABB RWS IP address.")
    parser.add_argument("--rws-port", default="80", help="ABB RWS HTTP port.")
    parser.add_argument("--rws-user", default=DEFAULT_RWS_USER, help="ABB RWS digest username.")
    parser.add_argument("--rws-password", default=DEFAULT_RWS_PASSWORD, help="ABB RWS digest password.")
    parser.add_argument("--rws-timeout-s", type=float, default=1.5, help="RWS read timeout.")
    parser.add_argument("--remote-host", default=DEFAULT_REMOTE_HOST, help="Tailscale remote inference host.")
    parser.add_argument("--hold-policy-health-url", default=DEFAULT_HOLD_POLICY_URL)
    parser.add_argument("--official-policy-health-url", default=DEFAULT_OFFICIAL_POLICY_URL)
    parser.add_argument("--front-camera-device", default=DEFAULT_FRONT_CAMERA_DEVICE)
    parser.add_argument("--wrist-camera-device", default=DEFAULT_WRIST_CAMERA_DEVICE)
    parser.add_argument("--camera-width", type=int, default=424)
    parser.add_argument("--camera-height", type=int, default=240)
    parser.add_argument("--camera-jpeg-quality", type=int, default=80)
    parser.add_argument("--workcell-urdf", default=DEFAULT_WORKCELL_URDF)
    parser.add_argument("--urdf-base-link", default=DEFAULT_URDF_BASE_LINK)
    parser.add_argument("--urdf-tip-link", default=DEFAULT_URDF_TIP_LINK)
    parser.add_argument("--workspace-radius-m", type=float, default=DEFAULT_WORKSPACE_RADIUS_M)
    parsed = parser.parse_args(args=args)

    config = make_dashboard_config(
        rws_ip=parsed.rws_ip,
        rws_port=parsed.rws_port,
        rws_user=parsed.rws_user,
        rws_password=parsed.rws_password,
        rws_timeout_s=parsed.rws_timeout_s,
        remote_host=parsed.remote_host,
        hold_policy_health_url=parsed.hold_policy_health_url,
        official_policy_health_url=parsed.official_policy_health_url,
        front_camera_device=parsed.front_camera_device,
        wrist_camera_device=parsed.wrist_camera_device,
        camera_width=parsed.camera_width,
        camera_height=parsed.camera_height,
        camera_jpeg_quality=parsed.camera_jpeg_quality,
        workcell_urdf=parsed.workcell_urdf,
        urdf_base_link=parsed.urdf_base_link,
        urdf_tip_link=parsed.urdf_tip_link,
        workspace_radius_m=parsed.workspace_radius_m,
    )

    server = ThreadingHTTPServer((parsed.host, parsed.port), DashboardHandler)
    server.dashboard_config = config
    print(f"ws_RAMS read-only dashboard: http://{parsed.host}:{parsed.port}")
    print("This tool only reads RWS/device/policy status. It does not command the robot.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
