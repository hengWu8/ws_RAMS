from __future__ import annotations

import base64
from dataclasses import replace
import json
import math

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

from .bridge_core import (
    CameraFrame,
    PolicyObservation,
    SafetyConfig,
    apply_safety_filters,
    build_observation,
    observation_is_fresh,
    observation_timestamps_aligned,
)
from .cartesian_workspace import UrdfForwardKinematics, distance_m
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
        self.feedback_joint_state_topic = str(
            self.declare_parameter("feedback_joint_state_topic", "").value
        )
        self.feedback_joint_state_timeout_sec = float(
            self.declare_parameter("feedback_joint_state_timeout_sec", 1.0).value
        )
        self.step_reference_mode = str(
            self.declare_parameter("step_reference_mode", "current_observation").value
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
        cartesian_test_direction_xyz = tuple(
            float(value)
            for value in self.declare_parameter(
                "cartesian_test_direction_xyz",
                [0.0, 1.0, 0.0],
            ).value
        )
        cartesian_test_direction_scalar_override = (
            float(self.declare_parameter("cartesian_test_direction_x", float("nan")).value),
            float(self.declare_parameter("cartesian_test_direction_y", float("nan")).value),
            float(self.declare_parameter("cartesian_test_direction_z", float("nan")).value),
        )
        if all(math.isfinite(value) for value in cartesian_test_direction_scalar_override):
            cartesian_test_direction_xyz = cartesian_test_direction_scalar_override
        self.cartesian_test_direction_xyz = cartesian_test_direction_xyz
        self.cartesian_test_step_m = float(
            self.declare_parameter("cartesian_test_step_m", 0.001).value
        )
        self.cartesian_test_speed_mps = float(
            self.declare_parameter("cartesian_test_speed_mps", 0.001).value
        )
        self.cartesian_test_tracking_gain = float(
            self.declare_parameter("cartesian_test_tracking_gain", 1.0).value
        )
        self.cartesian_test_max_cartesian_error_m = float(
            self.declare_parameter("cartesian_test_max_cartesian_error_m", 0.02).value
        )
        self.cartesian_test_damping = float(
            self.declare_parameter("cartesian_test_damping", 0.05).value
        )
        self.cartesian_test_fd_epsilon_rad = float(
            self.declare_parameter("cartesian_test_fd_epsilon_rad", 1e-4).value
        )
        self.cartesian_test_max_joint_delta_rad = float(
            self.declare_parameter("cartesian_test_max_joint_delta_rad", 0.005).value
        )
        self.enable_front_camera_observation = bool(
            self.declare_parameter("enable_front_camera_observation", False).value
        )
        self.enable_wrist_camera_observation = bool(
            self.declare_parameter("enable_wrist_camera_observation", False).value
        )
        self.front_camera_image_topic = str(
            self.declare_parameter(
                "front_camera_image_topic", "/camera/fixed_cam/color/image_raw"
            ).value
        )
        self.wrist_camera_image_topic = str(
            self.declare_parameter(
                "wrist_camera_image_topic", "/camera/wrist_cam/color/image_raw"
            ).value
        )
        self.camera_image_timeout_sec = float(
            self.declare_parameter("camera_image_timeout_sec", 0.5).value
        )
        self.max_camera_joint_skew_sec = float(
            self.declare_parameter("max_camera_joint_skew_sec", 0.2).value
        )
        self.auto_disarm_on_observation_fault = bool(
            self.declare_parameter("auto_disarm_on_observation_fault", True).value
        )
        self.camera_output_width = int(
            self.declare_parameter("camera_output_width", 224).value
        )
        self.camera_output_height = int(
            self.declare_parameter("camera_output_height", 224).value
        )
        self.camera_jpeg_quality = int(
            self.declare_parameter("camera_jpeg_quality", 80).value
        )
        self.robot_description = str(self.declare_parameter("robot_description", "").value)
        self.enable_cartesian_workspace_guard = bool(
            self.declare_parameter("enable_cartesian_workspace_guard", True).value
        )
        self.cartesian_workspace_radius_m = float(
            self.declare_parameter("cartesian_workspace_radius_m", 0.5).value
        )
        self.cartesian_workspace_base_link = str(
            self.declare_parameter("cartesian_workspace_base_link", "base_link").value
        )
        self.cartesian_workspace_tip_link = str(
            self.declare_parameter("cartesian_workspace_tip_link", "tcp").value
        )
        self.cartesian_workspace_reset_on_streaming_activate = bool(
            self.declare_parameter("cartesian_workspace_reset_on_streaming_activate", True).value
        )
        self.cartesian_workspace_reset_on_arm = bool(
            self.declare_parameter("cartesian_workspace_reset_on_arm", True).value
        )
        self.auto_disarm_on_workspace_violation = bool(
            self.declare_parameter("auto_disarm_on_workspace_violation", True).value
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
        if self.cartesian_workspace_radius_m <= 0.0:
            raise ValueError("cartesian_workspace_radius_m must be positive.")

        self.safety_config = SafetyConfig(
            lower_limits=lower_limits,
            upper_limits=upper_limits,
            max_position_step_rad=max_position_step_rad,
        )
        self.cartesian_fk = self._build_cartesian_fk()
        self.cartesian_workspace_center_xyz = None
        self.latest_tcp_xyz = None
        self.latest_candidate_tcp_xyz = None
        self.latest_cartesian_workspace_distance_m = None
        self.cartesian_workspace_center_stamp_sec = None
        self.policy = build_policy_backend(
            self.policy_backend,
            policy_server_url=self.policy_server_url,
            policy_request_timeout_sec=self.policy_request_timeout_sec,
            cartesian_fk=self.cartesian_fk,
            feedback_observation_provider=self._get_feedback_observation_for_policy,
            cartesian_test_direction_xyz=self.cartesian_test_direction_xyz,
            cartesian_test_step_m=self.cartesian_test_step_m,
            cartesian_test_speed_mps=self.cartesian_test_speed_mps,
            cartesian_test_tracking_gain=self.cartesian_test_tracking_gain,
            cartesian_test_max_cartesian_error_m=self.cartesian_test_max_cartesian_error_m,
            cartesian_test_damping=self.cartesian_test_damping,
            cartesian_test_fd_epsilon_rad=self.cartesian_test_fd_epsilon_rad,
            cartesian_test_max_joint_delta_rad=self.cartesian_test_max_joint_delta_rad,
        )
        self.fallback_policy = MockHoldPolicy()
        self.latest_observation = None
        self.latest_feedback_observation = None
        self.last_safe_command_positions = None
        self.latest_camera_frames: dict[str, CameraFrame] = {}
        self.camera_topics_by_name = {
            "front": self.front_camera_image_topic,
            "wrist": self.wrist_camera_image_topic,
        }
        self.io_callback_group = ReentrantCallbackGroup()
        self.service_callback_group = MutuallyExclusiveCallbackGroup()
        self.control_callback_group = MutuallyExclusiveCallbackGroup()

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._on_joint_state,
            10,
            callback_group=self.io_callback_group,
        )
        self.feedback_joint_state_sub = None
        if self.feedback_joint_state_topic and self.feedback_joint_state_topic != self.joint_state_topic:
            self.feedback_joint_state_sub = self.create_subscription(
                JointState,
                self.feedback_joint_state_topic,
                self._on_feedback_joint_state,
                10,
                callback_group=self.io_callback_group,
            )
        self.front_camera_sub = None
        self.wrist_camera_sub = None
        if self.enable_front_camera_observation:
            self.front_camera_sub = self.create_subscription(
                Image,
                self.front_camera_image_topic,
                lambda msg: self._on_camera_image("front", self.front_camera_image_topic, msg),
                5,
                callback_group=self.io_callback_group,
            )
        if self.enable_wrist_camera_observation:
            self.wrist_camera_sub = self.create_subscription(
                Image,
                self.wrist_camera_image_topic,
                lambda msg: self._on_camera_image("wrist", self.wrist_camera_image_topic, msg),
                5,
                callback_group=self.io_callback_group,
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
            callback_group=self.service_callback_group,
        )
        self.activate_click_to_move_srv = self.create_service(
            Trigger,
            "~/activate_click_to_move_mode",
            self._handle_activate_click_to_move_mode,
            callback_group=self.service_callback_group,
        )
        self.activate_monitor_srv = self.create_service(
            Trigger,
            "~/activate_monitor_mode",
            self._handle_activate_monitor_mode,
            callback_group=self.service_callback_group,
        )
        self.set_streaming_arm_srv = self.create_service(
            SetBool,
            "~/set_streaming_arm",
            self._handle_set_streaming_arm,
            callback_group=self.service_callback_group,
        )
        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self._on_control_tick,
            callback_group=self.control_callback_group,
        )

        self._publish_control_mode()
        self.get_logger().info(
            "abb_pi0_bridge started with "
            f"backend={self.policy_backend}, "
            f"publish_commands={self.publish_commands}, "
            f"command_topic={self.command_topic}, "
            f"feedback_joint_state_topic={self.feedback_joint_state_topic or self.joint_state_topic}, "
            f"step_reference_mode={self.step_reference_mode}, "
            f"control_mode={self.control_mode.value}, "
            f"armed={self.command_output_armed}, "
            f"front_camera={self.enable_front_camera_observation}, "
            f"wrist_camera={self.enable_wrist_camera_observation}, "
            f"cartesian_workspace_guard={self.enable_cartesian_workspace_guard}, "
            f"cartesian_workspace_radius_m={self.cartesian_workspace_radius_m}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        receive_time_sec = self._now_sec()
        stamp_sec = receive_time_sec
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9

        try:
            self.latest_observation = build_observation(
                ordered_joint_names=self.joint_names,
                msg_joint_names=msg.name,
                positions=msg.position,
                velocities=msg.velocity,
                efforts=msg.effort,
                stamp_sec=stamp_sec,
            )
        except ValueError as exc:
            self.get_logger().warn(f"Failed to build observation from JointState: {exc}")
            return
        self._maybe_initialize_cartesian_workspace_center("first_joint_state")

    def _on_feedback_joint_state(self, msg: JointState) -> None:
        receive_time_sec = self._now_sec()
        stamp_sec = receive_time_sec
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9

        try:
            self.latest_feedback_observation = build_observation(
                ordered_joint_names=self.joint_names,
                msg_joint_names=msg.name,
                positions=msg.position,
                velocities=msg.velocity,
                efforts=msg.effort,
                stamp_sec=stamp_sec,
            )
        except ValueError as exc:
            self.get_logger().warn(f"Failed to build feedback observation from JointState: {exc}")
            return

    def _on_camera_image(self, camera_name: str, topic: str, msg: Image) -> None:
        receive_time_sec = self._now_sec()
        try:
            camera_frame = self._build_camera_frame(
                camera_name=camera_name,
                topic=topic,
                msg=msg,
                fallback_stamp_sec=receive_time_sec,
            )
        except ValueError as exc:
            self.get_logger().warn(
                f"Failed to encode {camera_name} camera image from {topic}: {exc}"
            )
            return

        self.latest_camera_frames[camera_name] = camera_frame

    def _on_control_tick(self) -> None:
        if self.latest_observation is None:
            self._handle_observation_fault("waiting_for_joint_state")
            return

        now_sec = self._now_sec()
        if not observation_is_fresh(
            now_sec=now_sec,
            observation_stamp_sec=self.latest_observation.stamp_sec,
            timeout_sec=self.joint_state_timeout_sec,
        ):
            self._handle_observation_fault("stale_joint_state")
            return

        policy_observation = self._build_policy_observation(now_sec)
        if policy_observation is None:
            return

        try:
            requested_positions = self.policy.compute_command(policy_observation)
            policy_source = self.policy_backend
            policy_error = None
        except ValueError as exc:
            if not self.fallback_to_mock_on_policy_error:
                self._publish_status("command_rejected", str(exc), self.policy_backend, str(exc))
                self.get_logger().warn(f"Bridge command rejected: {exc}")
                return
            requested_positions = self.fallback_policy.compute_command(policy_observation)
            policy_source = "mock_hold_fallback"
            policy_error = str(exc)

        try:
            step_reference_positions = self._get_step_reference_positions(policy_observation)
            safe_command = apply_safety_filters(
                current_positions=policy_observation.positions,
                requested_positions=requested_positions,
                safety_config=self.safety_config,
                step_reference_positions=step_reference_positions,
            )
        except ValueError as exc:
            self._publish_status("command_rejected", str(exc), policy_source, policy_error)
            self.get_logger().warn(f"Bridge command rejected: {exc}")
            return

        workspace_rejection_reason = self._check_cartesian_workspace_guard(
            current_observation=policy_observation,
            candidate_positions=safe_command.positions,
        )
        if workspace_rejection_reason is not None:
            if self.auto_disarm_on_workspace_violation and self.command_output_armed:
                self.command_output_armed = False
            self._publish_status(
                "command_rejected",
                workspace_rejection_reason,
                policy_source,
                policy_error,
            )
            self.get_logger().warn(f"Bridge command rejected: {workspace_rejection_reason}")
            return

        output_active = command_output_active(
            control_mode=self.control_mode,
            publish_commands=self.publish_commands,
            command_output_armed=self.command_output_armed,
        )

        command_msg = Float64MultiArray(data=list(safe_command.positions))
        self.last_safe_command_positions = tuple(float(value) for value in safe_command.positions)
        self.safe_command_pub.publish(command_msg)
        if output_active:
            self.command_pub.publish(command_msg)

        self._publish_status("command_ready", safe_command.reason, policy_source, policy_error)

    def _build_policy_observation(self, now_sec: float) -> PolicyObservation | None:
        observation = self.latest_observation
        if observation is None:
            return None

        required_camera_names = []
        if self.enable_front_camera_observation:
            required_camera_names.append("front")
        if self.enable_wrist_camera_observation:
            required_camera_names.append("wrist")

        if not required_camera_names:
            return observation

        camera_frames = []
        for camera_name in required_camera_names:
            latest_frame = self.latest_camera_frames.get(camera_name)
            if latest_frame is None:
                self._handle_observation_fault(f"waiting_for_{camera_name}_camera_image")
                return None
            if not observation_is_fresh(
                now_sec=now_sec,
                observation_stamp_sec=latest_frame.stamp_sec,
                timeout_sec=self.camera_image_timeout_sec,
            ):
                self._handle_observation_fault(f"stale_{camera_name}_camera_image")
                return None
            if not observation_timestamps_aligned(
                reference_stamp_sec=observation.stamp_sec,
                sample_stamp_sec=latest_frame.stamp_sec,
                tolerance_sec=self.max_camera_joint_skew_sec,
            ):
                self._handle_observation_fault(f"{camera_name}_camera_joint_skew_exceeded")
                return None
            camera_frames.append(latest_frame)

        return replace(observation, camera_frames=tuple(camera_frames))

    def _handle_observation_fault(self, state: str) -> None:
        if self.auto_disarm_on_observation_fault and self.command_output_armed:
            self.command_output_armed = False
            self.get_logger().warn(
                f"Observation fault '{state}' detected; streaming output has been disarmed."
            )
        self._publish_status(state, None)

    def _handle_activate_streaming_mode(self, request, response):
        del request
        self.control_mode = ControlMode.STREAMING
        if self.cartesian_workspace_reset_on_streaming_activate:
            self._reset_cartesian_workspace_center("streaming_activate")
        self._reset_step_reference("streaming_activate")
        self._reset_policy_reference("streaming_activate")
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
        self._reset_step_reference("activate_click_to_move")
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
        self._reset_step_reference("activate_monitor")
        self._publish_control_mode()
        response.success = True
        response.message = (
            f"Control mode set to '{self.control_mode.value}'. "
            "Streaming output has been disarmed."
        )
        return response

    def _handle_set_streaming_arm(self, request, response):
        self.command_output_armed = bool(request.data)
        if self.command_output_armed and self.cartesian_workspace_reset_on_arm:
            self._reset_cartesian_workspace_center("streaming_arm")
            self._reset_step_reference("streaming_arm")
            self._reset_policy_reference("streaming_arm")
        if not self.command_output_armed:
            self._reset_step_reference("streaming_disarm")
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
            "feedback_joint_state_topic": self.feedback_joint_state_topic or self.joint_state_topic,
            "feedback_joint_state_timeout_sec": self.feedback_joint_state_timeout_sec,
            "step_reference_mode": self.step_reference_mode,
            "command_topic": self.command_topic,
            "safety_reason": safety_reason,
            "camera_topics": {
                camera_name: self.camera_topics_by_name[camera_name]
                for camera_name in ("front", "wrist")
                if (
                    (camera_name == "front" and self.enable_front_camera_observation)
                    or (camera_name == "wrist" and self.enable_wrist_camera_observation)
                )
            },
            "camera_observation_enabled": {
                "front": self.enable_front_camera_observation,
                "wrist": self.enable_wrist_camera_observation,
            },
            "camera_frames_received": sorted(self.latest_camera_frames.keys()),
            "max_camera_joint_skew_sec": self.max_camera_joint_skew_sec,
            "auto_disarm_on_observation_fault": self.auto_disarm_on_observation_fault,
            "cartesian_test_policy": {
                "direction_xyz": self.cartesian_test_direction_xyz,
                "step_m": self.cartesian_test_step_m,
                "speed_mps": self.cartesian_test_speed_mps,
                "tracking_gain": self.cartesian_test_tracking_gain,
                "max_cartesian_error_m": self.cartesian_test_max_cartesian_error_m,
                "damping": self.cartesian_test_damping,
                "finite_difference_epsilon_rad": self.cartesian_test_fd_epsilon_rad,
                "max_joint_delta_rad": self.cartesian_test_max_joint_delta_rad,
            },
            "cartesian_workspace_guard": {
                "enabled": self.enable_cartesian_workspace_guard,
                "ready": self.cartesian_fk is not None,
                "base_link": self.cartesian_workspace_base_link,
                "tip_link": self.cartesian_workspace_tip_link,
                "radius_m": self.cartesian_workspace_radius_m,
                "center_xyz": self.cartesian_workspace_center_xyz,
                "center_stamp_sec": self.cartesian_workspace_center_stamp_sec,
                "latest_tcp_xyz": self.latest_tcp_xyz,
                "latest_candidate_tcp_xyz": self.latest_candidate_tcp_xyz,
                "latest_distance_m": self.latest_cartesian_workspace_distance_m,
                "reset_on_streaming_activate": self.cartesian_workspace_reset_on_streaming_activate,
                "reset_on_arm": self.cartesian_workspace_reset_on_arm,
                "auto_disarm_on_violation": self.auto_disarm_on_workspace_violation,
            },
        }
        if self.latest_observation is not None:
            payload["observation"] = self.latest_observation.as_policy_input()
        if self.latest_feedback_observation is not None:
            payload["feedback_observation"] = self.latest_feedback_observation.as_policy_input()
        debug_state = getattr(self.policy, "debug_state", None)
        if callable(debug_state):
            payload["policy_debug"] = debug_state()

        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _build_cartesian_fk(self):
        if not self.enable_cartesian_workspace_guard:
            return None
        try:
            return UrdfForwardKinematics.from_robot_description(
                self.robot_description,
                base_link=self.cartesian_workspace_base_link,
                tip_link=self.cartesian_workspace_tip_link,
            )
        except ValueError as exc:
            self.get_logger().error(
                "Cartesian workspace guard is enabled but FK could not be initialized: "
                f"{exc}. Commands from abb_pi0_bridge will be rejected until this is fixed."
            )
            return None

    def _reset_policy_reference(self, reason: str) -> None:
        reset = getattr(self.policy, "reset", None)
        if callable(reset):
            reset()
            self.get_logger().info(f"Policy reference reset ({reason})")

    def _maybe_initialize_cartesian_workspace_center(self, reason: str) -> None:
        if self.cartesian_workspace_center_xyz is None:
            self._reset_cartesian_workspace_center(reason)

    def _reset_cartesian_workspace_center(self, reason: str) -> bool:
        if not self.enable_cartesian_workspace_guard:
            return True
        if self.cartesian_fk is None:
            return False
        if self.latest_observation is None:
            return False
        try:
            center = self.cartesian_fk.compute_tip_position(
                self.latest_observation.joint_names,
                self.latest_observation.positions,
            )
        except ValueError as exc:
            self.get_logger().warn(
                f"Failed to reset Cartesian workspace center from current robot pose: {exc}"
            )
            return False

        self.cartesian_workspace_center_xyz = tuple(float(value) for value in center)
        self.latest_tcp_xyz = self.cartesian_workspace_center_xyz
        self.latest_candidate_tcp_xyz = None
        self.latest_cartesian_workspace_distance_m = 0.0
        self.cartesian_workspace_center_stamp_sec = self.latest_observation.stamp_sec
        self.get_logger().info(
            "Cartesian workspace guard center reset "
            f"({reason}) to {self.cartesian_workspace_center_xyz} in "
            f"{self.cartesian_workspace_base_link}->{self.cartesian_workspace_tip_link}; "
            f"radius={self.cartesian_workspace_radius_m} m"
        )
        return True

    def _check_cartesian_workspace_guard(
        self,
        *,
        current_observation: PolicyObservation,
        candidate_positions: tuple[float, ...],
    ) -> str | None:
        if not self.enable_cartesian_workspace_guard:
            return None
        if self.cartesian_fk is None:
            return "cartesian_workspace_guard_unavailable"
        if self.cartesian_workspace_center_xyz is None:
            if not self._reset_cartesian_workspace_center("pre_command"):
                return "cartesian_workspace_center_unavailable"

        assert self.cartesian_workspace_center_xyz is not None
        try:
            self.latest_tcp_xyz = self.cartesian_fk.compute_tip_position(
                current_observation.joint_names,
                current_observation.positions,
            )
            self.latest_candidate_tcp_xyz = self.cartesian_fk.compute_tip_position(
                current_observation.joint_names,
                candidate_positions,
            )
        except ValueError as exc:
            return f"cartesian_workspace_fk_error:{exc}"

        self.latest_cartesian_workspace_distance_m = distance_m(
            self.latest_candidate_tcp_xyz,
            self.cartesian_workspace_center_xyz,
        )
        if self.latest_cartesian_workspace_distance_m > self.cartesian_workspace_radius_m:
            return (
                "cartesian_workspace_radius_exceeded:"
                f"distance_m={self.latest_cartesian_workspace_distance_m:.4f},"
                f"limit_m={self.cartesian_workspace_radius_m:.4f},"
                f"center={self.cartesian_workspace_center_xyz},"
                f"candidate={self.latest_candidate_tcp_xyz}"
            )
        return None

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _get_feedback_observation_for_policy(self) -> PolicyObservation | None:
        observation = self.latest_feedback_observation
        if observation is None:
            if not self.feedback_joint_state_topic:
                return self.latest_observation
            return None

        now_sec = self._now_sec()
        if not observation_is_fresh(
            now_sec=now_sec,
            observation_stamp_sec=observation.stamp_sec,
            timeout_sec=self.feedback_joint_state_timeout_sec,
        ):
            return None
        return observation

    def _get_step_reference_positions(
        self, observation: PolicyObservation
    ) -> tuple[float, ...] | None:
        if self.step_reference_mode == "current_observation":
            return observation.positions
        if self.step_reference_mode == "approved_command":
            if self.last_safe_command_positions is not None:
                return self.last_safe_command_positions
            return observation.positions
        raise ValueError(
            "step_reference_mode must be 'current_observation' or 'approved_command'."
        )

    def _reset_step_reference(self, reason: str) -> None:
        if self.latest_observation is not None:
            self.last_safe_command_positions = tuple(
                float(value) for value in self.latest_observation.positions
            )
        else:
            self.last_safe_command_positions = None
        self.get_logger().info(f"Step reference reset ({reason})")

    def _build_camera_frame(
        self,
        *,
        camera_name: str,
        topic: str,
        msg: Image,
        fallback_stamp_sec: float,
    ) -> CameraFrame:
        try:
            import cv2
            import numpy as np
        except ImportError as exc:  # pragma: no cover - depends on runtime image stack
            raise ValueError("OpenCV and numpy are required for camera observation support.") from exc

        if msg.height <= 0 or msg.width <= 0:
            raise ValueError("Image height and width must be positive.")
        if self.camera_output_width <= 0 or self.camera_output_height <= 0:
            raise ValueError("camera_output_width and camera_output_height must be positive.")
        if not (1 <= self.camera_jpeg_quality <= 100):
            raise ValueError("camera_jpeg_quality must be in the range [1, 100].")

        channel_count_by_encoding = {
            "mono8": 1,
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
        }
        channel_count = channel_count_by_encoding.get(msg.encoding)
        if channel_count is None:
            raise ValueError(f"Unsupported image encoding '{msg.encoding}'.")

        row_width_bytes = msg.width * channel_count
        if msg.step < row_width_bytes:
            raise ValueError(
                f"Image step {msg.step} is smaller than expected row width {row_width_bytes}."
            )

        flat = np.frombuffer(msg.data, dtype=np.uint8)
        expected_size = msg.height * msg.step
        if flat.size < expected_size:
            raise ValueError(
                f"Image buffer length {flat.size} is smaller than expected {expected_size}."
            )

        image_with_padding = flat[:expected_size].reshape(msg.height, msg.step)
        image = image_with_padding[:, :row_width_bytes]

        if msg.encoding == "mono8":
            decoded = image.reshape(msg.height, msg.width)
            image_bgr = cv2.cvtColor(decoded, cv2.COLOR_GRAY2BGR)
        else:
            decoded = image.reshape(msg.height, msg.width, channel_count)
            if msg.encoding == "rgb8":
                image_bgr = cv2.cvtColor(decoded, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "bgr8":
                image_bgr = decoded
            elif msg.encoding == "rgba8":
                image_bgr = cv2.cvtColor(decoded, cv2.COLOR_RGBA2BGR)
            else:
                image_bgr = cv2.cvtColor(decoded, cv2.COLOR_BGRA2BGR)

        if (
            image_bgr.shape[1] != self.camera_output_width
            or image_bgr.shape[0] != self.camera_output_height
        ):
            interpolation = (
                cv2.INTER_AREA
                if image_bgr.shape[1] >= self.camera_output_width
                and image_bgr.shape[0] >= self.camera_output_height
                else cv2.INTER_LINEAR
            )
            image_bgr = cv2.resize(
                image_bgr,
                (self.camera_output_width, self.camera_output_height),
                interpolation=interpolation,
            )

        success, encoded = cv2.imencode(
            ".jpg",
            image_bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(self.camera_jpeg_quality)],
        )
        if not success:
            raise ValueError("cv2.imencode('.jpg', ...) returned failure.")

        stamp_sec = fallback_stamp_sec
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9

        return CameraFrame(
            name=camera_name,
            stamp_sec=stamp_sec,
            frame_id=msg.header.frame_id,
            topic=topic,
            width=int(image_bgr.shape[1]),
            height=int(image_bgr.shape[0]),
            mime_type="image/jpeg",
            data_b64=base64.b64encode(encoded.tobytes()).decode("ascii"),
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AbbPi0BridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            # Launch shutdown sequence may have already finalized the context.
            pass
