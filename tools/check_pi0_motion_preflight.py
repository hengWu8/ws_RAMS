#!/usr/bin/env python3
"""Read-only preflight checks before allowing pi0 to command the ABB robot."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import urllib.error
import urllib.request

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import workcell_dashboard as dashboard


def run_command(cmd: list[str], timeout_sec: float = 5.0) -> tuple[int, str, str]:
    completed = subprocess.run(
        cmd,
        text=True,
        capture_output=True,
        timeout=timeout_sec,
        check=False,
    )
    return completed.returncode, completed.stdout.strip(), completed.stderr.strip()


def fetch_json(url: str, timeout_sec: float) -> tuple[bool, dict | None, str]:
    try:
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(url, timeout=timeout_sec) as response:
            body = response.read().decode("utf-8", errors="replace")
            return True, json.loads(body), body
    except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
        return False, None, str(exc)


def read_status_topic(topic: str, timeout_sec: float) -> tuple[bool, dict | None, str]:
    cmd = [
        "ros2",
        "topic",
        "echo",
        topic,
        "std_msgs/msg/String",
        "--once",
        "--field",
        "data",
        "--full-length",
        "--qos-reliability",
        "reliable",
        "--qos-durability",
        "volatile",
    ]
    rc, stdout, stderr = run_command(cmd, timeout_sec)
    if rc != 0:
        return False, None, stderr or stdout
    payload = stdout.strip()
    if "\n---" in payload:
        payload = payload.split("\n---", 1)[0].strip()
    if len(payload) >= 2 and payload[0] == payload[-1] and payload[0] in {"'", '"'}:
        payload = payload[1:-1]
    try:
        return True, json.loads(payload), payload
    except json.JSONDecodeError as exc:
        return False, None, f"invalid status JSON: {exc}\n{payload}"


def topic_info(topic: str, timeout_sec: float) -> tuple[int | None, int | None, str]:
    rc, stdout, stderr = run_command(["ros2", "topic", "info", "-v", topic], timeout_sec)
    if rc != 0:
        return None, None, stderr or stdout
    publishers = None
    subscribers = None
    for line in stdout.splitlines():
        stripped = line.strip()
        if stripped.startswith("Publisher count:"):
            publishers = int(stripped.split(":", 1)[1].strip())
        if stripped.startswith("Subscription count:"):
            subscribers = int(stripped.split(":", 1)[1].strip())
    return publishers, subscribers, stdout


def print_check(label: str, ok: bool, detail: str) -> None:
    print(f"[{'PASS' if ok else 'FAIL'}] {label}")
    if detail:
        print(detail)


def _finite_joint_vector(values) -> list[float] | None:
    if not isinstance(values, list) or not values:
        return None
    result = []
    for value in values:
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(numeric):
            return None
        result.append(numeric)
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Read-only pi0 motion preflight check.")
    parser.add_argument("--policy-health-url", default="http://100.70.7.8:8002/healthz")
    parser.add_argument("--bridge-status-topic", default="/abb_pi0_bridge/status")
    parser.add_argument("--command-topic", default="/forward_command_controller_position/commands")
    parser.add_argument("--egm-port", type=int, default=6515)
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    parser.add_argument("--rws-ip", default=os.environ.get("ABB_RWS_IP", dashboard.DEFAULT_RWS_IP))
    parser.add_argument("--rws-port", default="80")
    parser.add_argument("--rws-user", default=dashboard.DEFAULT_RWS_USER)
    parser.add_argument("--rws-password", default=dashboard.DEFAULT_RWS_PASSWORD)
    args = parser.parse_args(argv)

    if shutil.which("ros2") is None:
        print("ERROR: ros2 not found in PATH.", file=sys.stderr)
        return 2

    overall_ok = True

    health_ok, health_json, health_text = fetch_json(args.policy_health_url, args.timeout_sec)
    infer_stats = ((health_json or {}).get("infer_stats") or {}) if health_json else {}
    infer_rate = float(infer_stats.get("recent_success_hz_30s", 0.0) or 0.0)
    recent_success_count = int(infer_stats.get("recent_success_count_30s", 0) or 0)
    check_ok = health_ok and bool(health_json and health_json.get("policy_loaded"))
    overall_ok &= check_ok
    print_check(
        "remote pi05 ready",
        check_ok,
        health_text,
    )

    status_ok, status_json, status_text = read_status_topic(args.bridge_status_topic, args.timeout_sec)
    overall_ok &= status_ok
    print_check("bridge status readable", status_ok, status_text if not status_ok else "")

    if status_json:
        camera_frames = status_json.get("camera_frames_received") or []
        workspace = status_json.get("cartesian_workspace_guard") or {}
        latest_distance = float(workspace.get("latest_distance_m", math.inf) or math.inf)
        radius_m = float(workspace.get("radius_m", 0.0) or 0.0)
        publish_disabled = not bool(status_json.get("publish_commands"))
        output_inactive = not bool(status_json.get("command_output_active"))
        output_disarmed = not bool(status_json.get("command_output_armed"))
        workspace_ready = bool(workspace.get("enabled")) and bool(workspace.get("ready")) and radius_m <= 0.5
        workspace_inside = latest_distance <= radius_m if math.isfinite(latest_distance) else False
        cameras_ok = "front" in camera_frames and "wrist" in camera_frames
        state_ok = status_json.get("state") == "command_ready"

        overall_ok &= publish_disabled and output_inactive and output_disarmed
        overall_ok &= workspace_ready and workspace_inside and cameras_ok and state_ok

        print_check(
            "publish gate still closed",
            publish_disabled and output_inactive and output_disarmed,
            json.dumps(
                {
                    "publish_commands": status_json.get("publish_commands"),
                    "command_output_active": status_json.get("command_output_active"),
                    "command_output_armed": status_json.get("command_output_armed"),
                },
                indent=2,
                ensure_ascii=False,
            ),
        )
        print_check(
            "workspace guard ready",
            workspace_ready and workspace_inside,
            json.dumps(
                {
                    "enabled": workspace.get("enabled"),
                    "ready": workspace.get("ready"),
                    "radius_m": radius_m,
                    "latest_distance_m": latest_distance,
                    "center_xyz": workspace.get("center_xyz"),
                    "latest_candidate_tcp_xyz": workspace.get("latest_candidate_tcp_xyz"),
                },
                indent=2,
                ensure_ascii=False,
            ),
        )
        print_check(
            "observation path ready",
            cameras_ok and state_ok,
            json.dumps(
                {
                    "state": status_json.get("state"),
                    "camera_frames_received": camera_frames,
                    "policy_source": status_json.get("policy_source"),
                    "safety_reason": status_json.get("safety_reason"),
                },
                indent=2,
                ensure_ascii=False,
            ),
        )

        ros_joint_positions = _finite_joint_vector((status_json.get("observation") or {}).get("joint_positions"))
        rws_config = dashboard.make_dashboard_config(
            rws_ip=args.rws_ip,
            rws_port=args.rws_port,
            rws_user=args.rws_user,
            rws_password=args.rws_password,
        )
        rws_jointtarget = dashboard.read_rws_jointtarget(rws_config)
        rws_joint_positions = _finite_joint_vector(rws_jointtarget.get("radians"))
        joint_feedback_ok = False
        joint_feedback_detail = json.dumps(
            {
                "ros_joint_positions": ros_joint_positions,
                "rws_joint_positions": rws_joint_positions,
                "rws_ok": rws_jointtarget.get("ok"),
            },
            indent=2,
            ensure_ascii=False,
        )
        if ros_joint_positions is not None and rws_joint_positions is not None:
            max_abs_diff = max(abs(a - b) for a, b in zip(ros_joint_positions, rws_joint_positions))
            ros_nonzero = any(abs(value) > 1e-6 for value in ros_joint_positions)
            rws_nonzero = any(abs(value) > 1e-6 for value in rws_joint_positions)
            joint_feedback_ok = max_abs_diff <= 0.05 or (not rws_nonzero and not ros_nonzero)
            joint_feedback_detail = json.dumps(
                {
                    "ros_joint_positions": ros_joint_positions,
                    "rws_joint_positions": rws_joint_positions,
                    "max_abs_diff_rad": max_abs_diff,
                    "ros_nonzero": ros_nonzero,
                    "rws_nonzero": rws_nonzero,
                },
                indent=2,
                ensure_ascii=False,
            )
        overall_ok &= joint_feedback_ok
        print_check("joint feedback matches ABB RWS", joint_feedback_ok, joint_feedback_detail)

    publishers, subscribers, raw = topic_info(args.command_topic, args.timeout_sec)
    command_ok = publishers is not None and publishers > 0 and subscribers is not None and subscribers > 0
    overall_ok &= command_ok
    print_check(
        "command topic connected",
        command_ok,
        f"topic={args.command_topic}\npublishers={publishers} subscribers={subscribers}\n{raw}",
    )

    rc, stdout, stderr = run_command(["ss", "-lunap"], args.timeout_sec)
    egm_lines = [line for line in stdout.splitlines() if f":{args.egm_port}" in line] if rc == 0 else []
    egm_ok = bool(egm_lines)
    overall_ok &= egm_ok
    print_check("EGM UDP listener", egm_ok, "\n".join(egm_lines) if egm_lines else (stderr or stdout))

    infer_ok = recent_success_count > 0 and infer_rate > 0.0
    overall_ok &= infer_ok
    print_check(
        "recent pi05 inference traffic",
        infer_ok,
        json.dumps(
            {
                "recent_success_count_30s": recent_success_count,
                "recent_success_hz_30s": infer_rate,
                "last_success_infer_ms": infer_stats.get("last_success_infer_ms"),
                "success_count_total": infer_stats.get("success_count_total"),
            },
            indent=2,
            ensure_ascii=False,
        ),
    )

    return 0 if overall_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
