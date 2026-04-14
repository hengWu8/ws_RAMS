from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rws_ip",
                default_value="192.168.125.1",
                description="ABB RWS IP reachable from this ROS 2 machine. This workcell's verified controller address is 192.168.125.1.",
            ),
            DeclareLaunchArgument(
                "rws_port",
                default_value="80",
                description="ABB RWS TCP port reachable from this ROS 2 machine.",
            ),
            DeclareLaunchArgument(
                "rws_user",
                default_value="Default User",
                description="ABB RWS username used by the local true-state publisher.",
            ),
            DeclareLaunchArgument(
                "rws_password",
                default_value="robotics",
                description="ABB RWS password used by the local true-state publisher.",
            ),
            DeclareLaunchArgument(
                "egm_port",
                default_value="6515",
                description="ABB EGM UDP port exposed by the controller or RobotStudio VC.",
            ),
            DeclareLaunchArgument(
                "initial_joint_controller",
                default_value="forward_command_controller_position",
                description="Initial ros2_control command controller for pi0 streaming. Forward controller avoids joint-trajectory hold goals on startup.",
            ),
            DeclareLaunchArgument(
                "fixed_serial",
                default_value="_109522062115",
                description="Serial number of the fixed RealSense camera.",
            ),
            DeclareLaunchArgument(
                "fixed_usb_port_id",
                default_value="",
                description="USB port id of the fixed RealSense camera, for example 4.4.",
            ),
            DeclareLaunchArgument(
                "wrist_serial",
                default_value="_342522070232",
                description="Serial number of the wrist RealSense camera.",
            ),
            DeclareLaunchArgument(
                "wrist_usb_port_id",
                default_value="",
                description="USB port id of the wrist RealSense camera, for example 4.3.",
            ),
            DeclareLaunchArgument(
                "policy_server_url",
                default_value="http://100.70.7.8:8002/infer",
                description="Remote pi0/openpi HTTP endpoint on the Tailscale-connected inference machine.",
            ),
            DeclareLaunchArgument(
                "policy_backend",
                default_value="http_json",
                description="abb_pi0_bridge policy backend: http_json, mock_hold, cartesian_left_test, cartesian_left_demo_traj, or cartesian_left_true_servo.",
            ),
            DeclareLaunchArgument(
                "publish_commands",
                default_value="false",
                description="Whether abb_pi0_bridge is allowed to publish low-level commands.",
            ),
            DeclareLaunchArgument(
                "launch_moveit_rviz",
                default_value="false",
                description="Whether to launch the MoveIt RViz UI. Keep false on headless deployments.",
            ),
            DeclareLaunchArgument(
                "control_mode",
                default_value="trajectory",
                description="Initial abb_pi0_bridge control mode.",
            ),
            DeclareLaunchArgument(
                "bridge_policy_request_timeout_sec",
                default_value="0.5",
                description="HTTP timeout in seconds for the remote pi0 policy server.",
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
                "bridge_joint_state_timeout_sec",
                default_value="1.0",
                description="Maximum allowed age of ABB RWS-backed joint states before pi0 output is disarmed.",
            ),
            DeclareLaunchArgument(
                "bridge_max_position_step_rad",
                default_value="0.02",
                description="Per-tick joint step clamp inside abb_pi0_bridge safety filtering.",
            ),
            DeclareLaunchArgument(
                "cartesian_test_step_m",
                default_value="0.001",
                description="TCP step in meters per bridge tick for policy_backend=cartesian_left_test.",
            ),
            DeclareLaunchArgument(
                "cartesian_test_speed_mps",
                default_value="0.001",
                description="Cartesian demo trajectory speed in meters per second for policy_backend=cartesian_left_demo_traj.",
            ),
            DeclareLaunchArgument(
                "cartesian_test_tracking_gain",
                default_value="1.0",
                description="Closed-loop Cartesian tracking gain for policy_backend=cartesian_left_true_servo.",
            ),
            DeclareLaunchArgument(
                "cartesian_test_max_cartesian_error_m",
                default_value="0.02",
                description="Maximum Cartesian tracking error magnitude for policy_backend=cartesian_left_true_servo.",
            ),
            DeclareLaunchArgument(
                "cartesian_test_max_joint_delta_rad",
                default_value="0.005",
                description="Per-joint policy delta clamp for policy_backend=cartesian_left_test.",
            ),
            DeclareLaunchArgument(
                "bridge_joint_state_topic",
                default_value="/abb_rws/joint_states",
                description="JointState topic consumed by abb_pi0_bridge. Defaults to the RWS-backed true-state publisher.",
            ),
            DeclareLaunchArgument(
                "bridge_feedback_joint_state_topic",
                default_value="/abb_rws/joint_states",
                description="Optional true-state JointState topic used by closed-loop Cartesian demo policies.",
            ),
            DeclareLaunchArgument(
                "bridge_feedback_joint_state_timeout_sec",
                default_value="1.0",
                description="Maximum allowed age of the true-state JointState topic used by closed-loop Cartesian demo policies.",
            ),
            DeclareLaunchArgument(
                "bridge_step_reference_mode",
                default_value="current_observation",
                description="Joint step-clamp reference mode inside abb_pi0_bridge: current_observation or approved_command.",
            ),
            DeclareLaunchArgument(
                "launch_rws_state_publisher",
                default_value="true",
                description="Whether to launch the ABB RWS-backed true-state publisher.",
            ),
            DeclareLaunchArgument(
                "rws_joint_state_topic",
                default_value="/abb_rws/joint_states",
                description="ABB RWS-backed JointState topic published locally.",
            ),
            DeclareLaunchArgument(
                "rws_polling_rate",
                default_value="10.0",
                description="Polling rate in Hz for the ABB RWS-backed state publisher.",
            ),
            DeclareLaunchArgument(
                "cartesian_workspace_radius_m",
                default_value="0.5",
                description="Maximum allowed TCP distance from the pi0 handoff pose, in meters.",
            ),
            DeclareLaunchArgument(
                "bridge_enable_front_camera_observation",
                default_value="true",
                description="Whether abb_pi0_bridge includes the fixed camera image in policy requests.",
            ),
            DeclareLaunchArgument(
                "bridge_enable_wrist_camera_observation",
                default_value="true",
                description="Whether abb_pi0_bridge includes the wrist camera image in policy requests.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("click_to_move"), "launch", "system_bringup.launch.py"]
                    )
                ),
                launch_arguments={
                    "rws_ip": LaunchConfiguration("rws_ip"),
                    "rws_port": LaunchConfiguration("rws_port"),
                    "rws_user": LaunchConfiguration("rws_user"),
                    "rws_password": LaunchConfiguration("rws_password"),
                    "egm_port": LaunchConfiguration("egm_port"),
                    "initial_joint_controller": LaunchConfiguration("initial_joint_controller"),
                    "fixed_serial": LaunchConfiguration("fixed_serial"),
                    "fixed_usb_port_id": LaunchConfiguration("fixed_usb_port_id"),
                    "wrist_serial": LaunchConfiguration("wrist_serial"),
                    "wrist_usb_port_id": LaunchConfiguration("wrist_usb_port_id"),
                    "launch_moveit_rviz": LaunchConfiguration("launch_moveit_rviz"),
                    "launch_rws_state_publisher": LaunchConfiguration("launch_rws_state_publisher"),
                    "rws_joint_state_topic": LaunchConfiguration("rws_joint_state_topic"),
                    "rws_polling_rate": LaunchConfiguration("rws_polling_rate"),
                    "launch_bridge": "true",
                    "bridge_policy_backend": LaunchConfiguration("policy_backend"),
                    "bridge_policy_server_url": LaunchConfiguration("policy_server_url"),
                    "bridge_publish_commands": LaunchConfiguration("publish_commands"),
                    "bridge_control_mode": LaunchConfiguration("control_mode"),
                    "bridge_policy_request_timeout_sec": LaunchConfiguration(
                        "bridge_policy_request_timeout_sec"
                    ),
                    "bridge_cartesian_test_direction_x": LaunchConfiguration(
                        "cartesian_test_direction_x"
                    ),
                    "bridge_cartesian_test_direction_y": LaunchConfiguration(
                        "cartesian_test_direction_y"
                    ),
                    "bridge_cartesian_test_direction_z": LaunchConfiguration(
                        "cartesian_test_direction_z"
                    ),
                    "bridge_joint_state_timeout_sec": LaunchConfiguration(
                        "bridge_joint_state_timeout_sec"
                    ),
                    "bridge_max_position_step_rad": LaunchConfiguration(
                        "bridge_max_position_step_rad"
                    ),
                    "bridge_cartesian_test_step_m": LaunchConfiguration(
                        "cartesian_test_step_m"
                    ),
                    "bridge_cartesian_test_speed_mps": LaunchConfiguration(
                        "cartesian_test_speed_mps"
                    ),
                    "bridge_cartesian_test_tracking_gain": LaunchConfiguration(
                        "cartesian_test_tracking_gain"
                    ),
                    "bridge_cartesian_test_max_cartesian_error_m": LaunchConfiguration(
                        "cartesian_test_max_cartesian_error_m"
                    ),
                    "bridge_cartesian_test_max_joint_delta_rad": LaunchConfiguration(
                        "cartesian_test_max_joint_delta_rad"
                    ),
                    "bridge_joint_state_topic": LaunchConfiguration("bridge_joint_state_topic"),
                    "bridge_feedback_joint_state_topic": LaunchConfiguration(
                        "bridge_feedback_joint_state_topic"
                    ),
                    "bridge_feedback_joint_state_timeout_sec": LaunchConfiguration(
                        "bridge_feedback_joint_state_timeout_sec"
                    ),
                    "bridge_step_reference_mode": LaunchConfiguration(
                        "bridge_step_reference_mode"
                    ),
                    "bridge_enable_front_camera_observation": LaunchConfiguration(
                        "bridge_enable_front_camera_observation"
                    ),
                    "bridge_enable_wrist_camera_observation": LaunchConfiguration(
                        "bridge_enable_wrist_camera_observation"
                    ),
                    "bridge_front_camera_image_topic": "/camera/fixed_cam/color/image_raw",
                    "bridge_wrist_camera_image_topic": "/camera/wrist_cam/color/image_raw",
                    "bridge_camera_image_timeout_sec": "0.5",
                    "bridge_max_camera_joint_skew_sec": "1.0",
                    "bridge_auto_disarm_on_observation_fault": "true",
                    "bridge_enable_cartesian_workspace_guard": "true",
                    "bridge_cartesian_workspace_radius_m": LaunchConfiguration(
                        "cartesian_workspace_radius_m"
                    ),
                    "bridge_cartesian_workspace_base_link": "base_link",
                    "bridge_cartesian_workspace_tip_link": "tcp",
                    "bridge_cartesian_workspace_reset_on_streaming_activate": "true",
                    "bridge_cartesian_workspace_reset_on_arm": "true",
                    "bridge_auto_disarm_on_workspace_violation": "true",
                }.items(),
            ),
        ]
    )
