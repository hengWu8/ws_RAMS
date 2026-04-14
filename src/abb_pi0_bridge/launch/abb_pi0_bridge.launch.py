from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    publish_commands = LaunchConfiguration("publish_commands")
    control_rate_hz = LaunchConfiguration("control_rate_hz")
    command_topic = LaunchConfiguration("command_topic")
    control_mode = LaunchConfiguration("control_mode")
    policy_backend = LaunchConfiguration("policy_backend")
    policy_server_url = LaunchConfiguration("policy_server_url")
    enable_front_camera_observation = LaunchConfiguration("enable_front_camera_observation")
    enable_wrist_camera_observation = LaunchConfiguration("enable_wrist_camera_observation")
    front_camera_image_topic = LaunchConfiguration("front_camera_image_topic")
    wrist_camera_image_topic = LaunchConfiguration("wrist_camera_image_topic")
    policy_request_timeout_sec = LaunchConfiguration("policy_request_timeout_sec")
    cartesian_test_direction_x = LaunchConfiguration("cartesian_test_direction_x")
    cartesian_test_direction_y = LaunchConfiguration("cartesian_test_direction_y")
    cartesian_test_direction_z = LaunchConfiguration("cartesian_test_direction_z")
    camera_image_timeout_sec = LaunchConfiguration("camera_image_timeout_sec")
    max_camera_joint_skew_sec = LaunchConfiguration("max_camera_joint_skew_sec")
    auto_disarm_on_observation_fault = LaunchConfiguration("auto_disarm_on_observation_fault")
    robot_description = LaunchConfiguration("robot_description")
    enable_cartesian_workspace_guard = LaunchConfiguration("enable_cartesian_workspace_guard")
    cartesian_workspace_radius_m = LaunchConfiguration("cartesian_workspace_radius_m")
    cartesian_workspace_base_link = LaunchConfiguration("cartesian_workspace_base_link")
    cartesian_workspace_tip_link = LaunchConfiguration("cartesian_workspace_tip_link")
    cartesian_workspace_reset_on_streaming_activate = LaunchConfiguration(
        "cartesian_workspace_reset_on_streaming_activate"
    )
    cartesian_workspace_reset_on_arm = LaunchConfiguration("cartesian_workspace_reset_on_arm")
    auto_disarm_on_workspace_violation = LaunchConfiguration("auto_disarm_on_workspace_violation")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("abb_pi0_bridge"), "config", "pi0_bridge.params.yaml"]
                ),
                description="Path to the abb_pi0_bridge parameter file.",
            ),
            DeclareLaunchArgument(
                "publish_commands",
                default_value="false",
                description="Whether to publish filtered commands to the low-level controller topic.",
            ),
            DeclareLaunchArgument(
                "control_rate_hz",
                default_value="10.0",
                description="Control loop rate for the bridge node.",
            ),
            DeclareLaunchArgument(
                "command_topic",
                default_value="/forward_command_controller_position/commands",
                description="Low-level controller command topic.",
            ),
            DeclareLaunchArgument(
                "control_mode",
                default_value="trajectory",
                description="Initial shared control mode: trajectory, streaming, or monitor.",
            ),
            DeclareLaunchArgument(
                "policy_backend",
                default_value="mock_hold",
                description="Policy backend to use: mock_hold or http_json.",
            ),
            DeclareLaunchArgument(
                "policy_server_url",
                default_value="",
                description="HTTP endpoint for the remote policy stub when policy_backend=http_json.",
            ),
            DeclareLaunchArgument(
                "enable_front_camera_observation",
                default_value="false",
                description="Whether to include the fixed/front camera image in HTTP policy requests.",
            ),
            DeclareLaunchArgument(
                "enable_wrist_camera_observation",
                default_value="false",
                description="Whether to include the wrist camera image in HTTP policy requests.",
            ),
            DeclareLaunchArgument(
                "front_camera_image_topic",
                default_value="/camera/fixed_cam/color/image_raw",
                description="ROS image topic for the fixed/front camera.",
            ),
            DeclareLaunchArgument(
                "wrist_camera_image_topic",
                default_value="/camera/wrist_cam/color/image_raw",
                description="ROS image topic for the wrist camera.",
            ),
            DeclareLaunchArgument(
                "policy_request_timeout_sec",
                default_value="0.2",
                description="HTTP policy request timeout in seconds.",
            ),
            DeclareLaunchArgument(
                "cartesian_test_direction_x",
                default_value="0.0",
                description="Cartesian test direction X component in base frame.",
            ),
            DeclareLaunchArgument(
                "cartesian_test_direction_y",
                default_value="1.0",
                description="Cartesian test direction Y component in base frame.",
            ),
            DeclareLaunchArgument(
                "cartesian_test_direction_z",
                default_value="0.0",
                description="Cartesian test direction Z component in base frame.",
            ),
            DeclareLaunchArgument(
                "camera_image_timeout_sec",
                default_value="0.5",
                description="Maximum allowed age of camera frames in seconds.",
            ),
            DeclareLaunchArgument(
                "max_camera_joint_skew_sec",
                default_value="0.2",
                description="Maximum allowed timestamp skew between joint state and camera frames in seconds.",
            ),
            DeclareLaunchArgument(
                "auto_disarm_on_observation_fault",
                default_value="true",
                description="Automatically disarm streaming output when required observations become stale or misaligned.",
            ),
            DeclareLaunchArgument(
                "robot_description",
                default_value="",
                description="Expanded URDF XML used by the Cartesian workspace guard.",
            ),
            DeclareLaunchArgument(
                "enable_cartesian_workspace_guard",
                default_value="true",
                description="Reject bridge commands whose TCP target leaves the configured Cartesian radius.",
            ),
            DeclareLaunchArgument(
                "cartesian_workspace_radius_m",
                default_value="0.5",
                description="Maximum allowed TCP distance from the guard center in meters.",
            ),
            DeclareLaunchArgument(
                "cartesian_workspace_base_link",
                default_value="base_link",
                description="Base link for Cartesian workspace FK.",
            ),
            DeclareLaunchArgument(
                "cartesian_workspace_tip_link",
                default_value="tcp",
                description="Tip link for Cartesian workspace FK.",
            ),
            DeclareLaunchArgument(
                "cartesian_workspace_reset_on_streaming_activate",
                default_value="true",
                description="Reset the workspace center to the current TCP pose when streaming mode is activated.",
            ),
            DeclareLaunchArgument(
                "cartesian_workspace_reset_on_arm",
                default_value="true",
                description="Reset the workspace center to the current TCP pose when bridge output is armed.",
            ),
            DeclareLaunchArgument(
                "auto_disarm_on_workspace_violation",
                default_value="true",
                description="Automatically disarm bridge output if a command violates the Cartesian workspace guard.",
            ),
            Node(
                package="abb_pi0_bridge",
                executable="abb_pi0_bridge_node",
                name="abb_pi0_bridge",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "publish_commands": publish_commands,
                        "control_rate_hz": control_rate_hz,
                        "command_topic": command_topic,
                        "control_mode": control_mode,
                        "policy_backend": policy_backend,
                        "policy_server_url": policy_server_url,
                        "policy_request_timeout_sec": policy_request_timeout_sec,
                        "cartesian_test_direction_x": cartesian_test_direction_x,
                        "cartesian_test_direction_y": cartesian_test_direction_y,
                        "cartesian_test_direction_z": cartesian_test_direction_z,
                        "enable_front_camera_observation": enable_front_camera_observation,
                        "enable_wrist_camera_observation": enable_wrist_camera_observation,
                        "front_camera_image_topic": front_camera_image_topic,
                        "wrist_camera_image_topic": wrist_camera_image_topic,
                        "camera_image_timeout_sec": camera_image_timeout_sec,
                        "max_camera_joint_skew_sec": max_camera_joint_skew_sec,
                        "auto_disarm_on_observation_fault": auto_disarm_on_observation_fault,
                        "robot_description": robot_description,
                        "enable_cartesian_workspace_guard": enable_cartesian_workspace_guard,
                        "cartesian_workspace_radius_m": cartesian_workspace_radius_m,
                        "cartesian_workspace_base_link": cartesian_workspace_base_link,
                        "cartesian_workspace_tip_link": cartesian_workspace_tip_link,
                        "cartesian_workspace_reset_on_streaming_activate": cartesian_workspace_reset_on_streaming_activate,
                        "cartesian_workspace_reset_on_arm": cartesian_workspace_reset_on_arm,
                        "auto_disarm_on_workspace_violation": auto_disarm_on_workspace_violation,
                    },
                ],
            ),
        ]
    )
