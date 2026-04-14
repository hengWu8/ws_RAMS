#!/usr/bin/env python3
"""Continuously seed a ros2_control forward position controller with current joints.

This is a safety helper for ABB EGM bringup/tests. The ABB hardware interface may
begin sending the forward command controller's last command as soon as RAPID EGM
enters EGM_RUNNING. If that controller has not yet received a command matching
the robot's current joint position, the robot can be pulled toward a stale or
default target before the higher-level bridge is armed.

This tool subscribes to a trusted JointState source and republishes the latest
ordered joint positions as a Float64MultiArray hold command.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from typing import Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


DEFAULT_JOINT_NAMES = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")


class ForwardPositionHoldSeeder(Node):
    def __init__(
        self,
        *,
        joint_state_topic: str,
        command_topic: str,
        joint_names: Sequence[str],
        rate_hz: float,
        initial_positions: Sequence[float] | None,
    ) -> None:
        super().__init__("seed_forward_position_hold")
        self.joint_names = tuple(joint_names)
        self.latest_positions: tuple[float, ...] | None = (
            tuple(float(value) for value in initial_positions)
            if initial_positions is not None
            else None
        )
        self.publisher = self.create_publisher(Float64MultiArray, command_topic, 10)
        self.create_subscription(JointState, joint_state_topic, self._on_joint_state, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._publish_hold)
        self.get_logger().info(
            f"Seeding {command_topic} from {joint_state_topic} for joints={self.joint_names}"
        )
        if self.latest_positions is not None:
            self.get_logger().info("Using initial hold positions until JointState samples arrive.")

    def _on_joint_state(self, msg: JointState) -> None:
        index_by_name = {name: index for index, name in enumerate(msg.name)}
        positions = []
        for joint_name in self.joint_names:
            index = index_by_name.get(joint_name)
            if index is None or index >= len(msg.position):
                return
            value = float(msg.position[index])
            if not math.isfinite(value):
                return
            positions.append(value)
        self.latest_positions = tuple(positions)

    def _publish_hold(self) -> None:
        if self.latest_positions is None:
            return
        self.publisher.publish(Float64MultiArray(data=list(self.latest_positions)))


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--joint-state-topic", default="/abb_rws/joint_states")
    parser.add_argument("--command-topic", default="/forward_command_controller_position/commands")
    parser.add_argument("--joint-names", default=",".join(DEFAULT_JOINT_NAMES))
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--startup-timeout-sec", type=float, default=5.0)
    parser.add_argument("--duration-sec", type=float, default=0.0)
    parser.add_argument(
        "--positions-rad",
        default="",
        help=(
            "Optional comma-separated initial joint positions in radians. "
            "When set, publishing can start before the JointState topic exists."
        ),
    )
    args = parser.parse_args(argv)

    joint_names = tuple(name.strip() for name in args.joint_names.split(",") if name.strip())
    if not joint_names:
        raise SystemExit("--joint-names must contain at least one joint name")
    if args.rate_hz <= 0.0:
        raise SystemExit("--rate-hz must be positive")
    initial_positions = None
    if args.positions_rad.strip():
        initial_positions = tuple(
            float(value.strip()) for value in args.positions_rad.split(",") if value.strip()
        )
        if len(initial_positions) != len(joint_names):
            raise SystemExit(
                "--positions-rad must contain the same number of values as --joint-names"
            )
        if not all(math.isfinite(value) for value in initial_positions):
            raise SystemExit("--positions-rad values must be finite")

    rclpy.init()
    node = ForwardPositionHoldSeeder(
        joint_state_topic=args.joint_state_topic,
        command_topic=args.command_topic,
        joint_names=joint_names,
        rate_hz=args.rate_hz,
        initial_positions=initial_positions,
    )
    should_stop = False

    def _stop(_signum, _frame) -> None:
        nonlocal should_stop
        should_stop = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    start = time.monotonic()
    while rclpy.ok() and node.latest_positions is None:
        if time.monotonic() - start > args.startup_timeout_sec:
            node.get_logger().error("Timed out waiting for a complete JointState sample.")
            node.destroy_node()
            rclpy.shutdown()
            return 1
        rclpy.spin_once(node, timeout_sec=0.1)

    end_time = None if args.duration_sec <= 0.0 else time.monotonic() + args.duration_sec
    while rclpy.ok() and not should_stop:
        if end_time is not None and time.monotonic() >= end_time:
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
