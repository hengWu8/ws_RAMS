from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "workcell", package_name="workcell_config"
    ).to_moveit_configs()

    arguments = [
        DeclareLaunchArgument(
            "rws_ip",
            default_value="192.168.125.1",
            description="RWS IP reachable from the ROS2 machine. This workcell's verified controller address is 192.168.125.1.",
        ),
        DeclareLaunchArgument(
            "rws_port",
            default_value="80",
            description="RWS TCP port reachable from the ROS2 machine.",
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
            description="EGM UDP port exposed by the ABB controller or RobotStudio VC.",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Initial ros2_control command controller. Use forward_command_controller_position for pi0 streaming bringup.",
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
            "fixed_tf_parent_frame",
            default_value="fixed_cam_mount",
            description="Parent frame used for the fixed camera static TF.",
        ),
        DeclareLaunchArgument(
            "fixed_tf_child_frame",
            default_value="fixed_cam_link",
            description="Child frame used for the fixed camera static TF.",
        ),
        DeclareLaunchArgument(
            "fixed_tf_x",
            default_value="0.0",
            description="Fixed camera X offset from fixed_tf_parent_frame in meters.",
        ),
        DeclareLaunchArgument(
            "fixed_tf_y",
            default_value="0.0",
            description="Fixed camera Y offset from fixed_tf_parent_frame in meters.",
        ),
        DeclareLaunchArgument(
            "fixed_tf_z",
            default_value="0.0",
            description="Fixed camera Z offset from fixed_tf_parent_frame in meters.",
        ),
        DeclareLaunchArgument(
            "fixed_tf_roll",
            default_value="0.0",
            description="Fixed camera roll offset from fixed_tf_parent_frame in radians.",
        ),
        DeclareLaunchArgument(
            "fixed_tf_pitch",
            default_value="0.0",
            description="Fixed camera pitch offset from fixed_tf_parent_frame in radians.",
        ),
        DeclareLaunchArgument(
            "fixed_tf_yaw",
            default_value="0.0",
            description="Fixed camera yaw offset from fixed_tf_parent_frame in radians.",
        ),
        DeclareLaunchArgument(
            "wrist_tf_parent_frame",
            default_value="wrist_cam_mount",
            description="Parent frame used for the wrist camera static TF.",
        ),
        DeclareLaunchArgument(
            "wrist_tf_child_frame",
            default_value="wrist_cam_link",
            description="Child frame used for the wrist camera static TF.",
        ),
        DeclareLaunchArgument(
            "wrist_tf_x",
            default_value="0.0",
            description="Wrist camera X offset from wrist_tf_parent_frame in meters.",
        ),
        DeclareLaunchArgument(
            "wrist_tf_y",
            default_value="0.0",
            description="Wrist camera Y offset from wrist_tf_parent_frame in meters.",
        ),
        DeclareLaunchArgument(
            "wrist_tf_z",
            default_value="0.0",
            description="Wrist camera Z offset from wrist_tf_parent_frame in meters.",
        ),
        DeclareLaunchArgument(
            "wrist_tf_roll",
            default_value="0.0",
            description="Wrist camera roll offset from wrist_tf_parent_frame in radians.",
        ),
        DeclareLaunchArgument(
            "wrist_tf_pitch",
            default_value="0.0",
            description="Wrist camera pitch offset from wrist_tf_parent_frame in radians.",
        ),
        DeclareLaunchArgument(
            "wrist_tf_yaw",
            default_value="0.0",
            description="Wrist camera yaw offset from wrist_tf_parent_frame in radians.",
        ),
        DeclareLaunchArgument(
            "launch_bridge",
            default_value="false",
            description="Whether to launch abb_pi0_bridge together with the workcell bringup.",
        ),
        DeclareLaunchArgument(
            "launch_moveit_rviz",
            default_value="false",
            description="Whether to launch the MoveIt RViz UI. Keep false on headless machines.",
        ),
        DeclareLaunchArgument(
            "bridge_policy_backend",
            default_value="mock_hold",
            description="Policy backend used by abb_pi0_bridge: mock_hold, http_json, cartesian_left_test, cartesian_left_demo_traj, or cartesian_left_true_servo.",
        ),
        DeclareLaunchArgument(
            "bridge_policy_server_url",
            default_value="",
            description="Remote HTTP endpoint used by abb_pi0_bridge when bridge_policy_backend=http_json.",
        ),
        DeclareLaunchArgument(
            "bridge_publish_commands",
            default_value="false",
            description="Whether abb_pi0_bridge is allowed to publish low-level commands.",
        ),
        DeclareLaunchArgument(
            "bridge_control_mode",
            default_value="trajectory",
            description="Initial abb_pi0_bridge control mode.",
        ),
        DeclareLaunchArgument(
            "bridge_policy_request_timeout_sec",
            default_value="0.5",
            description="HTTP timeout in seconds for the remote policy server.",
        ),
        DeclareLaunchArgument(
            "bridge_cartesian_test_direction_x",
            default_value="0.0",
            description="Cartesian test direction X component in base frame.",
        ),
        DeclareLaunchArgument(
            "bridge_cartesian_test_direction_y",
            default_value="1.0",
            description="Cartesian test direction Y component in base frame.",
        ),
        DeclareLaunchArgument(
            "bridge_cartesian_test_direction_z",
            default_value="0.0",
            description="Cartesian test direction Z component in base frame.",
        ),
        DeclareLaunchArgument(
            "bridge_cartesian_test_step_m",
            default_value="0.001",
            description="TCP step in meters per bridge tick for policy_backend=cartesian_left_test.",
        ),
        DeclareLaunchArgument(
            "bridge_cartesian_test_speed_mps",
            default_value="0.001",
            description="Cartesian demo trajectory speed in meters per second for policy_backend=cartesian_left_demo_traj.",
        ),
        DeclareLaunchArgument(
            "bridge_cartesian_test_tracking_gain",
            default_value="1.0",
            description="Closed-loop Cartesian tracking gain for policy_backend=cartesian_left_true_servo.",
        ),
        DeclareLaunchArgument(
            "bridge_cartesian_test_max_cartesian_error_m",
            default_value="0.02",
            description="Maximum Cartesian error magnitude passed into the closed-loop demo policy.",
        ),
        DeclareLaunchArgument(
            "bridge_cartesian_test_max_joint_delta_rad",
            default_value="0.005",
            description="Per-joint policy delta clamp for policy_backend=cartesian_left_test.",
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
            "bridge_enable_front_camera_observation",
            default_value="true",
            description="Whether abb_pi0_bridge includes the fixed camera image in policy requests.",
        ),
        DeclareLaunchArgument(
            "bridge_enable_wrist_camera_observation",
            default_value="true",
            description="Whether abb_pi0_bridge includes the wrist camera image in policy requests.",
        ),
        DeclareLaunchArgument(
            "bridge_front_camera_image_topic",
            default_value="/camera/fixed_cam/color/image_raw",
            description="Fixed camera RGB topic forwarded to abb_pi0_bridge.",
        ),
        DeclareLaunchArgument(
            "bridge_wrist_camera_image_topic",
            default_value="/camera/wrist_cam/color/image_raw",
            description="Wrist camera RGB topic forwarded to abb_pi0_bridge.",
        ),
        DeclareLaunchArgument(
            "bridge_camera_image_timeout_sec",
            default_value="0.5",
            description="Maximum allowed age of camera frames in abb_pi0_bridge.",
        ),
        DeclareLaunchArgument(
            "bridge_max_camera_joint_skew_sec",
            default_value="1.0",
            description="Maximum allowed timestamp skew between camera frames and joint states in abb_pi0_bridge.",
        ),
        DeclareLaunchArgument(
            "bridge_auto_disarm_on_observation_fault",
            default_value="true",
            description="Automatically disarm abb_pi0_bridge output when observations go stale or misaligned.",
        ),
        DeclareLaunchArgument(
            "bridge_enable_cartesian_workspace_guard",
            default_value="true",
            description="Whether abb_pi0_bridge rejects TCP targets outside the local Cartesian safety radius.",
        ),
        DeclareLaunchArgument(
            "bridge_cartesian_workspace_radius_m",
            default_value="0.5",
            description="Maximum allowed TCP distance from the pi0 handoff pose, in meters.",
        ),
        DeclareLaunchArgument(
            "bridge_cartesian_workspace_base_link",
            default_value="base_link",
            description="Base link used by the abb_pi0_bridge Cartesian workspace guard.",
        ),
        DeclareLaunchArgument(
            "bridge_cartesian_workspace_tip_link",
            default_value="tcp",
            description="Tip link used by the abb_pi0_bridge Cartesian workspace guard.",
        ),
        DeclareLaunchArgument(
            "bridge_cartesian_workspace_reset_on_streaming_activate",
            default_value="true",
            description="Reset the Cartesian safety center when pi0 streaming mode is activated.",
        ),
        DeclareLaunchArgument(
            "bridge_cartesian_workspace_reset_on_arm",
            default_value="true",
            description="Reset the Cartesian safety center when pi0 output is armed.",
        ),
        DeclareLaunchArgument(
            "bridge_auto_disarm_on_workspace_violation",
            default_value="true",
            description="Automatically disarm abb_pi0_bridge if a command violates the Cartesian workspace guard.",
        ),
        DeclareLaunchArgument(
            "launch_rws_state_publisher",
            default_value="true",
            description="Whether to launch the ABB RWS-backed joint state publisher for true robot feedback.",
        ),
        DeclareLaunchArgument(
            "rws_joint_state_topic",
            default_value="/abb_rws/joint_states",
            description="Topic used for the ABB RWS-backed JointState stream.",
        ),
        DeclareLaunchArgument(
            "rws_polling_rate",
            default_value="10.0",
            description="Polling rate in Hz for the ABB RWS-backed state publisher.",
        ),
    ]

    fixed_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={
            "camera_name": "fixed_cam",
            "serial_no": LaunchConfiguration("fixed_serial"),
            "usb_port_id": LaunchConfiguration("fixed_usb_port_id"),
            "align_depth": "true",
            "pointcloud.enable": "true",
            "color_qos": "SENSOR_DATA",
            "depth_qos": "SENSOR_DATA",
        }.items(),
    )

    wrist_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={
            "camera_name": "wrist_cam",
            "serial_no": LaunchConfiguration("wrist_serial"),
            "usb_port_id": LaunchConfiguration("wrist_usb_port_id"),
            "align_depth": "true",
            "pointcloud.enable": "true",
            "color_qos": "SENSOR_DATA",
            "depth_qos": "SENSOR_DATA",
        }.items(),
    )

    tf_fixed = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_fixed_cam",
        arguments=[
            "--x",
            LaunchConfiguration("fixed_tf_x"),
            "--y",
            LaunchConfiguration("fixed_tf_y"),
            "--z",
            LaunchConfiguration("fixed_tf_z"),
            "--roll",
            LaunchConfiguration("fixed_tf_roll"),
            "--pitch",
            LaunchConfiguration("fixed_tf_pitch"),
            "--yaw",
            LaunchConfiguration("fixed_tf_yaw"),
            "--frame-id",
            LaunchConfiguration("fixed_tf_parent_frame"),
            "--child-frame-id",
            LaunchConfiguration("fixed_tf_child_frame"),
        ],
        output="screen",
    )

    tf_wrist = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_wrist_cam",
        arguments=[
            "--x",
            LaunchConfiguration("wrist_tf_x"),
            "--y",
            LaunchConfiguration("wrist_tf_y"),
            "--z",
            LaunchConfiguration("wrist_tf_z"),
            "--roll",
            LaunchConfiguration("wrist_tf_roll"),
            "--pitch",
            LaunchConfiguration("wrist_tf_pitch"),
            "--yaw",
            LaunchConfiguration("wrist_tf_yaw"),
            "--frame-id",
            LaunchConfiguration("wrist_tf_parent_frame"),
            "--child-frame-id",
            LaunchConfiguration("wrist_tf_child_frame"),
        ],
        output="screen",
    )

    abb_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("abb_bringup"), "launch", "abb_control.launch.py"]
            )
        ),
        launch_arguments={
            "description_package": "workcell_description",
            "description_file": "workcell.urdf.xacro",
            "use_fake_hardware": "false",
            "rws_ip": LaunchConfiguration("rws_ip"),
            "rws_port": LaunchConfiguration("rws_port"),
            "egm_port": LaunchConfiguration("egm_port"),
            "initial_joint_controller": LaunchConfiguration("initial_joint_controller"),
            "launch_rviz": "false",
        }.items(),
    )

    abb_rws_state = Node(
        package="abb_pi0_bridge",
        executable="rws_joint_state_publisher",
        name="abb_rws_state",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_rws_state_publisher")),
        parameters=[
            {
                "robot_ip": LaunchConfiguration("rws_ip"),
                "robot_port": LaunchConfiguration("rws_port"),
                "rws_user": LaunchConfiguration("rws_user"),
                "rws_password": LaunchConfiguration("rws_password"),
                "polling_rate": LaunchConfiguration("rws_polling_rate"),
            }
        ],
        remappings=[
            ("~/joint_states", LaunchConfiguration("rws_joint_state_topic")),
        ],
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("workcell_config"), "launch", "move_group.launch.py"]
            )
        ),
    )

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("workcell_config"), "launch", "moveit_rviz.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("launch_moveit_rviz")),
    )

    click_to_move = Node(
        package="click_to_move",
        executable="click_to_move_node",
        name="click_to_move",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "planning_group": "abb_irb6700",
                "base_frame": "base",
                "eef_link": "",
                "clicked_point_topic": "/clicked_point",
                "control_mode_topic": "/abb/control_mode",
                "required_control_mode": "trajectory",
                "default_control_mode": "trajectory",
                "above_offset": 0.05,
                "plan_only": False,
                "velocity_scaling": 0.2,
                "acceleration_scaling": 0.1,
                "use_current_orientation": True,
                "fallback_rpy": [3.14159, 0.0, 0.0],
            },
        ],
    )

    bridge = Node(
        package="abb_pi0_bridge",
        executable="abb_pi0_bridge_node",
        name="abb_pi0_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_bridge")),
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("abb_pi0_bridge"), "config", "pi0_bridge.params.yaml"]
            ),
            moveit_config.robot_description,
            {
                "publish_commands": LaunchConfiguration("bridge_publish_commands"),
                "control_mode": LaunchConfiguration("bridge_control_mode"),
                "policy_backend": LaunchConfiguration("bridge_policy_backend"),
                "policy_server_url": LaunchConfiguration("bridge_policy_server_url"),
                "policy_request_timeout_sec": LaunchConfiguration("bridge_policy_request_timeout_sec"),
                "cartesian_test_direction_x": LaunchConfiguration("bridge_cartesian_test_direction_x"),
                "cartesian_test_direction_y": LaunchConfiguration("bridge_cartesian_test_direction_y"),
                "cartesian_test_direction_z": LaunchConfiguration("bridge_cartesian_test_direction_z"),
                "cartesian_test_step_m": LaunchConfiguration("bridge_cartesian_test_step_m"),
                "cartesian_test_speed_mps": LaunchConfiguration("bridge_cartesian_test_speed_mps"),
                "cartesian_test_tracking_gain": LaunchConfiguration(
                    "bridge_cartesian_test_tracking_gain"
                ),
                "cartesian_test_max_cartesian_error_m": LaunchConfiguration(
                    "bridge_cartesian_test_max_cartesian_error_m"
                ),
                "cartesian_test_max_joint_delta_rad": LaunchConfiguration(
                    "bridge_cartesian_test_max_joint_delta_rad"
                ),
                "joint_state_timeout_sec": LaunchConfiguration("bridge_joint_state_timeout_sec"),
                "max_position_step_rad": LaunchConfiguration("bridge_max_position_step_rad"),
                "joint_state_topic": LaunchConfiguration("bridge_joint_state_topic"),
                "feedback_joint_state_topic": LaunchConfiguration(
                    "bridge_feedback_joint_state_topic"
                ),
                "feedback_joint_state_timeout_sec": LaunchConfiguration(
                    "bridge_feedback_joint_state_timeout_sec"
                ),
                "step_reference_mode": LaunchConfiguration("bridge_step_reference_mode"),
                "enable_front_camera_observation": LaunchConfiguration("bridge_enable_front_camera_observation"),
                "enable_wrist_camera_observation": LaunchConfiguration("bridge_enable_wrist_camera_observation"),
                "front_camera_image_topic": LaunchConfiguration("bridge_front_camera_image_topic"),
                "wrist_camera_image_topic": LaunchConfiguration("bridge_wrist_camera_image_topic"),
                "camera_image_timeout_sec": LaunchConfiguration("bridge_camera_image_timeout_sec"),
                "max_camera_joint_skew_sec": LaunchConfiguration("bridge_max_camera_joint_skew_sec"),
                "auto_disarm_on_observation_fault": LaunchConfiguration("bridge_auto_disarm_on_observation_fault"),
                "enable_cartesian_workspace_guard": LaunchConfiguration("bridge_enable_cartesian_workspace_guard"),
                "cartesian_workspace_radius_m": LaunchConfiguration("bridge_cartesian_workspace_radius_m"),
                "cartesian_workspace_base_link": LaunchConfiguration("bridge_cartesian_workspace_base_link"),
                "cartesian_workspace_tip_link": LaunchConfiguration("bridge_cartesian_workspace_tip_link"),
                "cartesian_workspace_reset_on_streaming_activate": LaunchConfiguration(
                    "bridge_cartesian_workspace_reset_on_streaming_activate"
                ),
                "cartesian_workspace_reset_on_arm": LaunchConfiguration(
                    "bridge_cartesian_workspace_reset_on_arm"
                ),
                "auto_disarm_on_workspace_violation": LaunchConfiguration(
                    "bridge_auto_disarm_on_workspace_violation"
                ),
            },
        ],
    )

    return LaunchDescription(
        arguments
        + [
            fixed_cam,
            wrist_cam,
            tf_fixed,
            tf_wrist,
            abb_control,
            abb_rws_state,
            move_group,
            moveit_rviz,
            click_to_move,
            bridge,
        ]
    )
