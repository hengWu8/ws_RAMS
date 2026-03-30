from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

from .bridge_core import SafetyConfig, apply_safety_filters, build_observation, observation_is_fresh
from .control_mode import ControlMode, command_output_active, parse_control_mode
from .mock_policy import MockHoldPolicy, build_policy_backend


class AbbPi0BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("abb_pi0_bridge")

        self.joint_names = tuple(
            self.declare_parameter(
                "joint_names",
                ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            ).value
        )
        lower_limits = tuple(
            float(value)
            for value in self.declare_parameter(
                "joint_position_lower_limits",
                [-6.283185307179586, -2.41, -6.283185307179586, -2.66, -6.283185307179586, -2.23],
            ).value
        )
        upper_limits = tuple(
            float(value)
            for value in self.declare_parameter(
                "joint_position_upper_limits",
                [6.283185307179586, 2.41, 6.283185307179586, 2.66, 6.283185307179586, 2.23],
            ).value
        )
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 10.0).value)
        self.joint_state_timeout_sec = float(
            self.declare_parameter("joint_state_timeout_sec", 0.5).value
        )
        self.publish_commands = bool(self.declare_parameter("publish_commands", False).value)
        self.command_output_armed = bool(
            self.declare_parameter("command_output_armed", False).value
        )
        self.arm_on_streaming_activate = bool(
            self.declare_parameter("arm_on_streaming_activate", False).value
        )
        self.joint_state_topic = str(
            self.declare_parameter("joint_state_topic", "/joint_states").value
        )
        self.command_topic = str(
            self.declare_parameter(
                "command_topic", "/forward_command_controller_position/commands"
            ).value
        )
        self.command_debug_topic = str(
            self.declare_parameter("command_debug_topic", "~/safe_command").value
        )
        self.status_topic = str(self.declare_parameter("status_topic", "~/status").value)
        self.control_mode_topic = str(
            self.declare_parameter("control_mode_topic", "/abb/control_mode").value
        )
        self.control_mode = parse_control_mode(
            str(self.declare_parameter("control_mode", "trajectory").value)
        )
        self.policy_backend = str(self.declare_parameter("policy_backend", "mock_hold").value)
        self.policy_server_url = str(self.declare_parameter("policy_server_url", "").value)
        self.policy_request_timeout_sec = float(
            self.declare_parameter("policy_request_timeout_sec", 0.2).value
        )
        self.fallback_to_mock_on_policy_error = bool(
            self.declare_parameter("fallback_to_mock_on_policy_error", True).value
        )
        max_position_step_rad = float(
            self.declare_parameter("max_position_step_rad", 0.02).value
        )

        if self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive.")
        if len(self.joint_names) != len(lower_limits):
            raise ValueError("joint_position_lower_limits length must match joint_names.")
        if len(self.joint_names) != len(upper_limits):
            raise ValueError("joint_position_upper_limits length must match joint_names.")

        self.safety_config = SafetyConfig(
            lower_limits=lower_limits,
            upper_limits=upper_limits,
            max_position_step_rad=max_position_step_rad,
        )
        self.policy = build_policy_backend(
            self.policy_backend,
            policy_server_url=self.policy_server_url,
            policy_request_timeout_sec=self.policy_request_timeout_sec,
        )
        self.fallback_policy = MockHoldPolicy()
        self.latest_observation = None

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._on_joint_state,
            10,
        )

        control_mode_qos = QoSProfile(depth=1)
        control_mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        control_mode_qos.reliability = ReliabilityPolicy.RELIABLE
        self.control_mode_pub = self.create_publisher(
            String, self.control_mode_topic, control_mode_qos
        )

        self.safe_command_pub = self.create_publisher(
            Float64MultiArray, self.command_debug_topic, 10
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.command_pub = self.create_publisher(Float64MultiArray, self.command_topic, 10)

        self.activate_streaming_srv = self.create_service(
            Trigger,
            "~/activate_streaming_mode",
            self._handle_activate_streaming_mode,
        )
        self.activate_click_to_move_srv = self.create_service(
            Trigger,
            "~/activate_click_to_move_mode",
            self._handle_activate_click_to_move_mode,
        )
        self.activate_monitor_srv = self.create_service(
            Trigger,
            "~/activate_monitor_mode",
            self._handle_activate_monitor_mode,
        )
        self.set_streaming_arm_srv = self.create_service(
            SetBool,
            "~/set_streaming_arm",
            self._handle_set_streaming_arm,
        )
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self._on_control_tick)

        self._publish_control_mode()
        self.get_logger().info(
            "abb_pi0_bridge started with "
            f"backend={self.policy_backend}, "
            f"publish_commands={self.publish_commands}, "
            f"command_topic={self.command_topic}, "
            f"control_mode={self.control_mode.value}, "
            f"armed={self.command_output_armed}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        receive_time_sec = self._now_sec()

        try:
            self.latest_observation = build_observation(
                ordered_joint_names=self.joint_names,
                msg_joint_names=msg.name,
                positions=msg.position,
                velocities=msg.velocity,
                efforts=msg.effort,
                stamp_sec=receive_time_sec,
            )
        except ValueError as exc:
            self.get_logger().warn(f"Failed to build observation from JointState: {exc}")
            return

    def _on_control_tick(self) -> None:
        if self.latest_observation is None:
            self._publish_status("waiting_for_joint_state", None)
            return

        now_sec = self._now_sec()
        if not observation_is_fresh(
            now_sec=now_sec,
            observation_stamp_sec=self.latest_observation.stamp_sec,
            timeout_sec=self.joint_state_timeout_sec,
        ):
            self._publish_status("stale_joint_state", None)
            return

        try:
            requested_positions = self.policy.compute_command(self.latest_observation)
            policy_source = self.policy_backend
            policy_error = None
        except ValueError as exc:
            if not self.fallback_to_mock_on_policy_error:
                self._publish_status("command_rejected", str(exc), self.policy_backend, str(exc))
                self.get_logger().warn(f"Bridge command rejected: {exc}")
                return
            requested_positions = self.fallback_policy.compute_command(self.latest_observation)
            policy_source = "mock_hold_fallback"
            policy_error = str(exc)

        try:
            safe_command = apply_safety_filters(
                current_positions=self.latest_observation.positions,
                requested_positions=requested_positions,
                safety_config=self.safety_config,
            )
        except ValueError as exc:
            self._publish_status("command_rejected", str(exc), policy_source, policy_error)
            self.get_logger().warn(f"Bridge command rejected: {exc}")
            return

        output_active = command_output_active(
            control_mode=self.control_mode,
            publish_commands=self.publish_commands,
            command_output_armed=self.command_output_armed,
        )

        command_msg = Float64MultiArray(data=list(safe_command.positions))
        self.safe_command_pub.publish(command_msg)
        if output_active:
            self.command_pub.publish(command_msg)

        self._publish_status("command_ready", safe_command.reason, policy_source, policy_error)

    def _handle_activate_streaming_mode(self, request, response):
        del request
        self.control_mode = ControlMode.STREAMING
        if self.arm_on_streaming_activate:
            self.command_output_armed = True
        self._publish_control_mode()
        response.success = True
        response.message = (
            f"Control mode set to '{self.control_mode.value}'. "
            f"command_output_armed={self.command_output_armed}"
        )
        return response

    def _handle_activate_click_to_move_mode(self, request, response):
        del request
        self.control_mode = ControlMode.TRAJECTORY
        self.command_output_armed = False
        self._publish_control_mode()
        response.success = True
        response.message = (
            f"Control mode set to '{self.control_mode.value}'. "
            "Streaming output has been disarmed."
        )
        return response

    def _handle_activate_monitor_mode(self, request, response):
        del request
        self.control_mode = ControlMode.MONITOR
        self.command_output_armed = False
        self._publish_control_mode()
        response.success = True
        response.message = (
            f"Control mode set to '{self.control_mode.value}'. "
            "Streaming output has been disarmed."
        )
        return response

    def _handle_set_streaming_arm(self, request, response):
        self.command_output_armed = bool(request.data)
        response.success = True
        response.message = (
            f"command_output_armed={self.command_output_armed}; "
            f"output_active={command_output_active(self.control_mode, self.publish_commands, self.command_output_armed)}"
        )
        return response

    def _publish_control_mode(self) -> None:
        self.control_mode_pub.publish(String(data=self.control_mode.value))

    def _publish_status(
        self,
        state: str,
        safety_reason: str | None,
        policy_source: str | None = None,
        policy_error: str | None = None,
    ) -> None:
        payload = {
            "state": state,
            "publish_commands": self.publish_commands,
            "command_output_armed": self.command_output_armed,
            "command_output_active": command_output_active(
                self.control_mode,
                self.publish_commands,
                self.command_output_armed,
            ),
            "control_mode": self.control_mode.value,
            "control_mode_topic": self.control_mode_topic,
            "policy_backend": self.policy_backend,
            "policy_source": policy_source,
            "policy_error": policy_error,
            "joint_state_topic": self.joint_state_topic,
            "command_topic": self.command_topic,
            "safety_reason": safety_reason,
        }
        if self.latest_observation is not None:
            payload["observation"] = self.latest_observation.as_policy_input()

        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AbbPi0BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
