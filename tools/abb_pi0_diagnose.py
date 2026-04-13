#!/usr/bin/env python3
"""Diagnostics for the ABB pi0 / EGM command path.

This script checks the three most likely bottlenecks for "commands are being
computed, but the robot does not move":

1. Bridge gating: is abb_pi0_bridge actually allowed to publish low-level
   commands?
2. ros2_control routing: is the expected controller active, and is anyone
   subscribed to the command topic?
3. EGM activity: is the ABB hardware interface bound on the configured UDP
   port, which is a prerequisite for RobotStudio EGM packets?

Typical usage:

  source /opt/ros/humble/setup.bash
  source /home/rob/workspace/ws_RAMS/install/setup.bash
  python3 tools/abb_pi0_diagnose.py

The script only reads ROS graph/runtime information. It does not modify any
controllers or send motion commands.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
from dataclasses import dataclass
from typing import Optional


@dataclass
class ControllerInfo:
    name: str
    type: str
    state: str


@dataclass
class TopicCounts:
    publishers: Optional[int]
    subscribers: Optional[int]
    raw: str


def run_command(cmd: list[str], timeout_sec: float = 5.0) -> tuple[int, str, str]:
    completed = subprocess.run(
        cmd,
        text=True,
        capture_output=True,
        timeout=timeout_sec,
        check=False,
    )
    return completed.returncode, completed.stdout.strip(), completed.stderr.strip()


def require_ros2() -> None:
    if shutil.which("ros2") is None:
        raise RuntimeError(
            "ros2 was not found in PATH. Source the ROS 2 workspace before running this script."
        )


def maybe_int(match: Optional[re.Match[str]]) -> Optional[int]:
    if match is None:
        return None
    return int(match.group(1))


def topic_info(topic: str) -> TopicCounts:
    rc, stdout, stderr = run_command(["ros2", "topic", "info", "-v", topic])
    if rc != 0:
        return TopicCounts(publishers=None, subscribers=None, raw=stderr or stdout)

    pub_match = re.search(r"Publisher count:\s*(\d+)", stdout)
    sub_match = re.search(r"Subscription count:\s*(\d+)", stdout)
    return TopicCounts(
        publishers=maybe_int(pub_match),
        subscribers=maybe_int(sub_match),
        raw=stdout,
    )


def echo_string_topic(topic: str, timeout_sec: float = 3.0) -> Optional[str]:
    rc, stdout, stderr = run_command(["ros2", "topic", "echo", "-n", "1", topic], timeout_sec)
    if rc != 0:
        return None

    data_line = None
    for line in stdout.splitlines():
        stripped = line.strip()
        if stripped.startswith("data:"):
            data_line = stripped[len("data:") :].strip()
            break

    if data_line is None:
        return stdout.strip() or None

    if len(data_line) >= 2 and data_line[0] == data_line[-1] and data_line[0] in {"'", '"'}:
        data_line = data_line[1:-1]

    return data_line


def list_controllers(controller_manager: str, timeout_sec: float = 5.0) -> list[ControllerInfo]:
    cmd = [
        "ros2",
        "control",
        "list_controllers",
        "-c",
        controller_manager,
    ]
    rc, stdout, stderr = run_command(cmd, timeout_sec)
    if rc != 0:
        raise RuntimeError(stderr or stdout or "Failed to run `ros2 control list_controllers`.")

    controllers: list[ControllerInfo] = []
    for line in stdout.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("name ") or stripped.startswith("---"):
            continue

        parts = stripped.split()
        if len(parts) < 3:
            continue
        controllers.append(ControllerInfo(name=parts[0], type=parts[1], state=parts[2]))
    return controllers


def check_egm_udp_port(port: int) -> tuple[bool, str]:
    if shutil.which("ss") is None:
        return False, "`ss` was not found in PATH, so UDP binding could not be checked."

    rc, stdout, stderr = run_command(["ss", "-lunap"])
    if rc != 0:
        return False, stderr or stdout or "Failed to run `ss -lunap`."

    matches = [line for line in stdout.splitlines() if f":{port}" in line]
    if not matches:
        return False, f"No local UDP listener was found on port {port}."

    return True, "\n".join(matches)


def print_section(title: str) -> None:
    print()
    print("=" * 72)
    print(title)
    print("=" * 72)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Diagnose ABB pi0 command delivery, ros2_control activation, and local EGM binding."
    )
    parser.add_argument(
        "--bridge-status-topic",
        default="/abb_pi0_bridge/status",
        help="abb_pi0_bridge status topic (std_msgs/String JSON payload).",
    )
    parser.add_argument(
        "--control-mode-topic",
        default="/abb/control_mode",
        help="Shared control mode topic used by abb_pi0_bridge and click_to_move.",
    )
    parser.add_argument(
        "--command-topic",
        default="/forward_command_controller_position/commands",
        help="Low-level command topic that should reach the ABB controller.",
    )
    parser.add_argument(
        "--controller-manager",
        default="/controller_manager",
        help="controller_manager namespace for ros2 control CLI.",
    )
    parser.add_argument(
        "--expected-controller",
        default="forward_command_controller_position",
        help="Controller that should consume the low-level streaming command topic.",
    )
    parser.add_argument(
        "--secondary-controller",
        default="joint_trajectory_controller",
        help="The trajectory controller often present in the same bringup.",
    )
    parser.add_argument(
        "--egm-port",
        type=int,
        default=6515,
        help="Local UDP port used by the ABB EGM hardware interface.",
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=3.0,
        help="Timeout for each ROS CLI query.",
    )
    args = parser.parse_args(argv)

    try:
        require_ros2()
    except RuntimeError as exc:
        print(f"ERROR: {exc}")
        return 2

    print("ABB pi0 / EGM diagnosis")
    print(f"ROS command topic: {args.command_topic}")
    print(f"Bridge status topic: {args.bridge_status_topic}")
    print(f"Control mode topic: {args.control_mode_topic}")
    print(f"controller_manager: {args.controller_manager}")
    print(f"EGM UDP port: {args.egm_port}")

    # 1) Bridge status and control mode
    print_section("1) Bridge gating")
    control_mode_value = echo_string_topic(args.control_mode_topic, args.timeout_sec)
    status_payload = echo_string_topic(args.bridge_status_topic, args.timeout_sec)
    control_mode_info = topic_info(args.control_mode_topic)
    status_info = topic_info(args.bridge_status_topic)

    print(
        f"control_mode_topic publishers={control_mode_info.publishers} subscribers={control_mode_info.subscribers}"
    )
    print(f"control_mode current value: {control_mode_value!r}")
    print(f"bridge status topic publishers={status_info.publishers} subscribers={status_info.subscribers}")

    status_json = None
    if status_payload is not None:
        try:
            status_json = json.loads(status_payload)
            print("bridge status payload parsed as JSON:")
            print(json.dumps(status_json, indent=2, sort_keys=True))
        except json.JSONDecodeError:
            print("bridge status payload (raw):")
            print(status_payload)
    else:
        print("bridge status payload: unavailable")

    if isinstance(status_json, dict):
        command_output_active = status_json.get("command_output_active")
        publish_commands = status_json.get("publish_commands")
        command_output_armed = status_json.get("command_output_armed")
        bridge_state = status_json.get("state")
        print(
            f"summary: state={bridge_state!r}, publish_commands={publish_commands}, "
            f"command_output_armed={command_output_armed}, command_output_active={command_output_active}"
        )
        if command_output_active is False:
            print(
                "likely bottleneck: abb_pi0_bridge is not allowed to publish low-level commands yet."
            )
            print(
                "check: control mode must be streaming, publish_commands must be true, and command_output_armed must be true."
            )

    # 2) ros2_control controller status
    print_section("2) ros2_control routing")
    try:
        controllers = list_controllers(args.controller_manager, args.timeout_sec)
    except Exception as exc:
        print(f"failed to list controllers: {exc}")
        controllers = []

    if controllers:
        print("controllers:")
        for controller in controllers:
            marker = ""
            if controller.name == args.expected_controller and controller.state != "active":
                marker = "  <-- expected low-level controller is not active"
            if controller.name == args.secondary_controller and controller.state == "active":
                marker = "  <-- trajectory controller active"
            print(f"  - {controller.name:32s} {controller.state:12s} {controller.type}{marker}")
    else:
        print("no controllers parsed from ros2 control output.")

    command_topic_info = topic_info(args.command_topic)
    print(
        f"command topic publishers={command_topic_info.publishers} subscribers={command_topic_info.subscribers}"
    )
    if command_topic_info.subscribers == 0:
        print(
            "likely bottleneck: the low-level command topic currently has zero subscribers, so no controller is consuming it."
        )

    expected_active = next((c for c in controllers if c.name == args.expected_controller), None)
    if expected_active is not None and expected_active.state != "active":
        print(
            f"likely bottleneck: {args.expected_controller} is {expected_active.state!r}, not active."
        )

    # 3) Local EGM binding
    print_section("3) EGM UDP binding")
    egm_bound, egm_detail = check_egm_udp_port(args.egm_port)
    print(egm_detail)
    if not egm_bound:
        print(
            "likely bottleneck: the ABB hardware interface is not listening on the EGM UDP port, or ros2_control never finished activating it."
        )

    # 4) Compact conclusion
    print_section("4) Quick read")
    issues: list[str] = []
    if isinstance(status_json, dict) and status_json.get("command_output_active") is not True:
        issues.append("bridge output is gated off")
    if expected_active is not None and expected_active.state != "active":
        issues.append(f"{args.expected_controller} is not active")
    if command_topic_info.subscribers == 0:
        issues.append("command topic has no subscriber")
    if not egm_bound:
        issues.append("EGM UDP port is not bound locally")

    if issues:
        print("main issues detected:")
        for issue in issues:
            print(f"  - {issue}")
    else:
        print(
            "No obvious local gating problem was found. If the robot still does not move, the next place to inspect is RobotStudio EGM/RAPID activity and the UDP path from RobotStudio to this machine."
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
