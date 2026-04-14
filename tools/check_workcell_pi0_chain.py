#!/usr/bin/env python3
"""Verify the local-camera -> bridge -> remote-pi0 -> ABB command chain.

This script is intentionally read-only. It does not arm motion output or publish
any robot commands. It only inspects:

1. Whether the local camera topics exist and are publishing.
2. Whether abb_pi0_bridge is publishing status and has received camera frames.
3. Whether the remote Tailscale pi0 policy server is reachable.
4. Whether the bridge is producing safe commands and whether streaming output is gated on/off.
5. Whether the ABB low-level command topic and EGM UDP listener look alive.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass


@dataclass
class CommandResult:
    rc: int
    stdout: str
    stderr: str


def run_command(cmd: list[str], timeout_sec: float = 5.0) -> CommandResult:
    completed = subprocess.run(
        cmd,
        text=True,
        capture_output=True,
        timeout=timeout_sec,
        check=False,
    )
    return CommandResult(
        rc=completed.returncode,
        stdout=completed.stdout.strip(),
        stderr=completed.stderr.strip(),
    )


def topic_info(topic: str, timeout_sec: float) -> tuple[int | None, int | None, str]:
    result = run_command(["ros2", "topic", "info", "-v", topic], timeout_sec)
    if result.rc != 0:
        return None, None, result.stderr or result.stdout

    pub_match = re.search(r"Publisher count:\s*(\d+)", result.stdout)
    sub_match = re.search(r"Subscription count:\s*(\d+)", result.stdout)
    publishers = int(pub_match.group(1)) if pub_match else None
    subscribers = int(sub_match.group(1)) if sub_match else None
    return publishers, subscribers, result.stdout


def topic_info_with_retries(topic: str, timeout_sec: float, retries: int, retry_delay_sec: float) -> tuple[int | None, int | None, str]:
    details: list[str] = []
    publishers = None
    subscribers = None
    for attempt in range(1, max(1, retries) + 1):
        publishers, subscribers, raw = topic_info(topic, timeout_sec)
        details.append(f"attempt={attempt}\n{raw}")
        if publishers is not None and publishers > 0:
            break
        if attempt < retries:
            time.sleep(max(0.0, retry_delay_sec))
    return publishers, subscribers, "\n\n".join(details)


def echo_once(topic: str, timeout_sec: float) -> str | None:
    result = run_command(
        [
            "ros2",
            "topic",
            "echo",
            "--once",
            "--field",
            "data",
            "--full-length",
            "--qos-reliability",
            "reliable",
            "--qos-durability",
            "volatile",
            topic,
            "std_msgs/msg/String",
        ],
        timeout_sec,
    )
    if result.rc != 0:
        return None
    return result.stdout or None


def string_topic_payload(topic: str, timeout_sec: float) -> str | None:
    output = echo_once(topic, timeout_sec)
    if not output:
        return None

    payload = output.strip()
    if "\n---" in payload:
        payload = payload.split("\n---", 1)[0].strip()
    if len(payload) >= 2 and payload[0] == payload[-1] and payload[0] in {"'", '"'}:
        payload = payload[1:-1]
    return payload or None


def policy_health_ok(url: str, timeout_sec: float) -> tuple[bool, str]:
    try:
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(url, timeout=timeout_sec) as response:
            body = response.read().decode("utf-8", errors="replace")
            return 200 <= response.status < 300, body
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        return False, str(exc)


def egm_port_info(port: int, timeout_sec: float) -> tuple[bool, str]:
    if shutil.which("ss") is None:
        return False, "`ss` not available"
    result = run_command(["ss", "-lunap"], timeout_sec)
    if result.rc != 0:
        return False, result.stderr or result.stdout
    matches = [line for line in result.stdout.splitlines() if f":{port}" in line]
    if not matches:
        return False, f"no UDP listener on :{port}"
    return True, "\n".join(matches)


def print_check(label: str, ok: bool, detail: str) -> None:
    prefix = "PASS" if ok else "FAIL"
    print(f"[{prefix}] {label}")
    if detail:
        print(detail)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Verify the local RealSense -> bridge -> remote pi0 -> ABB chain.")
    parser.add_argument("--front-topic", default="/camera/fixed_cam/color/image_raw")
    parser.add_argument("--wrist-topic", default="/camera/wrist_cam/color/image_raw")
    parser.add_argument("--bridge-status-topic", default="/abb_pi0_bridge/status")
    parser.add_argument("--safe-command-topic", default="/abb_pi0_bridge/safe_command")
    parser.add_argument("--command-topic", default="/forward_command_controller_position/commands")
    parser.add_argument("--expected-bridge-joint-state-topic", default="/abb_rws/joint_states")
    parser.add_argument("--policy-health-url", default="http://100.70.7.8:8002/healthz")
    parser.add_argument("--egm-port", type=int, default=6515)
    parser.add_argument("--timeout-sec", type=float, default=3.0)
    parser.add_argument("--topic-retries", type=int, default=3)
    parser.add_argument("--retry-delay-sec", type=float, default=0.6)
    args = parser.parse_args(argv)

    if shutil.which("ros2") is None:
        print("ERROR: ros2 not found in PATH. Source ROS 2 and this workspace first.", file=sys.stderr)
        return 2

    status_payload = string_topic_payload(args.bridge_status_topic, args.timeout_sec)
    status_ok = status_payload is not None
    status_detail = f"topic={args.bridge_status_topic}\npayload={status_payload}"
    status_json = None
    camera_frames_received: list[str] = []
    if status_payload is not None:
        try:
            status_json = json.loads(status_payload)
            camera_frames_received = status_json.get("camera_frames_received", [])
            status_ok = "front" in camera_frames_received and "wrist" in camera_frames_received
            status_detail = json.dumps(status_json, indent=2, sort_keys=True)
        except json.JSONDecodeError:
            status_ok = False
            status_detail = f"invalid JSON payload:\n{status_payload}"
    overall_ok = status_ok
    print_check("bridge received camera frames", status_ok, status_detail)

    bridge_joint_state_ok = False
    bridge_joint_state_detail = ""
    if isinstance(status_json, dict):
        configured_joint_state_topic = status_json.get("joint_state_topic")
        bridge_joint_state_ok = configured_joint_state_topic == args.expected_bridge_joint_state_topic
        bridge_joint_state_detail = json.dumps(
            {
                "bridge_joint_state_topic": configured_joint_state_topic,
                "expected_bridge_joint_state_topic": args.expected_bridge_joint_state_topic,
            },
            indent=2,
            ensure_ascii=False,
        )
        overall_ok &= bridge_joint_state_ok
    print_check("bridge joint feedback source", bridge_joint_state_ok, bridge_joint_state_detail)

    for camera_name, label, topic in (
        ("front", "front camera", args.front_topic),
        ("wrist", "wrist camera", args.wrist_topic),
    ):
        publishers, subscribers, raw = topic_info_with_retries(
            topic,
            args.timeout_sec,
            retries=args.topic_retries,
            retry_delay_sec=args.retry_delay_sec,
        )
        saw_bridge_frame = camera_name in camera_frames_received
        ok = (publishers is not None and publishers > 0) or saw_bridge_frame
        overall_ok &= ok
        print_check(
            label,
            ok,
            (
                f"topic={topic}\npublishers={publishers} subscribers={subscribers}\n"
                f"bridge_received={saw_bridge_frame}\n{raw}"
            ),
        )

    policy_ok, policy_detail = policy_health_ok(args.policy_health_url, args.timeout_sec)
    overall_ok &= policy_ok
    print_check("remote pi0 health", policy_ok, f"url={args.policy_health_url}\n{policy_detail}")

    safe_publishers, safe_subscribers, safe_raw = topic_info(args.safe_command_topic, args.timeout_sec)
    safe_ok = safe_publishers is not None and safe_publishers > 0
    overall_ok &= safe_ok
    print_check(
        "bridge safe command output",
        safe_ok,
        f"topic={args.safe_command_topic}\npublishers={safe_publishers} subscribers={safe_subscribers}\n{safe_raw}",
    )

    command_publishers, command_subscribers, command_raw = topic_info(args.command_topic, args.timeout_sec)
    command_ok = command_publishers is not None and command_publishers > 0
    print_check(
        "ABB low-level command topic",
        command_ok,
        f"topic={args.command_topic}\npublishers={command_publishers} subscribers={command_subscribers}\n{command_raw}",
    )

    egm_ok, egm_detail = egm_port_info(args.egm_port, args.timeout_sec)
    print_check("ABB EGM UDP listener", egm_ok, egm_detail)

    if isinstance(status_json, dict):
        output_active = bool(status_json.get("command_output_active"))
        state = status_json.get("state")
        policy_source = status_json.get("policy_source")
        print()
        print("Bridge summary:")
        print(json.dumps(
            {
                "state": state,
                "policy_source": policy_source,
                "command_output_active": output_active,
                "camera_frames_received": status_json.get("camera_frames_received"),
                "joint_state_topic": status_json.get("joint_state_topic"),
            },
            indent=2,
            sort_keys=True,
        ))

    return 0 if overall_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
