from __future__ import annotations

import json
import math

import requests
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class RwsJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("abb_rws_state")

        self.robot_ip = str(self.declare_parameter("robot_ip", "192.168.125.1").value)
        self.robot_port = int(str(self.declare_parameter("robot_port", 80).value))
        self.rws_user = str(self.declare_parameter("rws_user", "Default User").value)
        self.rws_password = str(self.declare_parameter("rws_password", "robotics").value)
        self.mechanical_unit = str(self.declare_parameter("mechanical_unit", "ROB_1").value)
        self.polling_rate = float(self.declare_parameter("polling_rate", 10.0).value)
        self.request_timeout_sec = float(self.declare_parameter("request_timeout_sec", 1.0).value)
        self.joint_names = list(
            self.declare_parameter(
                "joint_names",
                ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            ).value
        )

        if self.polling_rate <= 0.0:
            raise ValueError("polling_rate must be positive")
        if len(self.joint_names) != 6:
            raise ValueError("joint_names must contain six ABB robot joints")

        self.session = requests.Session()
        self.session.trust_env = False
        self.session.headers.update({"Connection": "keep-alive"})
        self.session.auth = requests.auth.HTTPDigestAuth(self.rws_user, self.rws_password)
        self.joint_state_pub = self.create_publisher(JointState, "~/joint_states", 10)
        self.timer = self.create_timer(1.0 / self.polling_rate, self._on_timer)
        self.failure_count = 0
        self.last_positions: list[float] | None = None

        self.get_logger().info(
            "ABB RWS joint state publisher started "
            f"for {self.robot_ip}:{self.robot_port}, topic={self.resolve_topic_name('~/joint_states')}"
        )

    def _on_timer(self) -> None:
        url = (
            f"http://{self.robot_ip}:{self.robot_port}/rw/motionsystem/mechunits/"
            f"{self.mechanical_unit}/jointtarget?json=1"
        )
        try:
            response = self.session.get(url, timeout=self.request_timeout_sec)
            body = response.text
            response.raise_for_status()
            payload = json.loads(body)
            positions = self._parse_joint_positions(payload)
        except (requests.RequestException, json.JSONDecodeError, ValueError) as exc:
            self.failure_count += 1
            if self.failure_count == 1 or self.failure_count % 20 == 0:
                self.get_logger().warn(f"RWS joint poll failed ({self.failure_count}): {exc}")
            return

        self.failure_count = 0
        self.last_positions = positions

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = positions
        self.joint_state_pub.publish(msg)

    @staticmethod
    def _parse_joint_positions(payload: dict) -> list[float]:
        try:
            state = payload["_embedded"]["_state"][0]
        except (KeyError, IndexError, TypeError) as exc:
            raise ValueError(f"jointtarget payload missing state: {exc}") from exc

        positions: list[float] = []
        for index in range(1, 7):
            try:
                degrees = float(state[f"rax_{index}"])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"jointtarget missing rax_{index}: {exc}") from exc
            positions.append(math.radians(degrees))
        return positions


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RwsJointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.session.close()
        node.destroy_node()
        rclpy.shutdown()
