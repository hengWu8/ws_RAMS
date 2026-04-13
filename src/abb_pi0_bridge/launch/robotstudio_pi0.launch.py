import importlib.util

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _package_exists(package_name: str) -> bool:
    try:
        get_package_share_directory(package_name)
        return True
    except PackageNotFoundError:
        return False


def _module_exists(module_name: str) -> bool:
    return importlib.util.find_spec(module_name) is not None


def generate_launch_description():
    robotstudio_rws_ip = LaunchConfiguration("robotstudio_rws_ip")
    robotstudio_rws_port = LaunchConfiguration("robotstudio_rws_port")
    robotstudio_rws_username = LaunchConfiguration("robotstudio_rws_username")
    robotstudio_rws_password = LaunchConfiguration("robotstudio_rws_password")
    robot_controller_id = LaunchConfiguration("robot_controller_id")
    egm_port = LaunchConfiguration("egm_port")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_moveit = LaunchConfiguration("launch_moveit")
    launch_click_to_move = LaunchConfiguration("launch_click_to_move")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_policy_stub = LaunchConfiguration("launch_policy_stub")
    policy_server_host = LaunchConfiguration("policy_server_host")
    policy_server_port = LaunchConfiguration("policy_server_port")
    policy_stub_mode = LaunchConfiguration("policy_stub_mode")
    policy_stub_step_rad = LaunchConfiguration("policy_stub_step_rad")
    policy_stub_joint_index = LaunchConfiguration("policy_stub_joint_index")
    bridge_params_file = LaunchConfiguration("bridge_params_file")
    policy_server_url = LaunchConfiguration("policy_server_url")
    publish_commands = LaunchConfiguration("publish_commands")
    control_mode = LaunchConfiguration("control_mode")
    enable_front_camera_observation = LaunchConfiguration("enable_front_camera_observation")
    enable_wrist_camera_observation = LaunchConfiguration("enable_wrist_camera_observation")
    front_camera_image_topic = LaunchConfiguration("front_camera_image_topic")
    wrist_camera_image_topic = LaunchConfiguration("wrist_camera_image_topic")
    policy_request_timeout_sec = LaunchConfiguration("policy_request_timeout_sec")
    camera_image_timeout_sec = LaunchConfiguration("camera_image_timeout_sec")
    max_camera_joint_skew_sec = LaunchConfiguration("max_camera_joint_skew_sec")
    auto_disarm_on_observation_fault = LaunchConfiguration("auto_disarm_on_observation_fault")
    enable_cartesian_workspace_guard = LaunchConfiguration("enable_cartesian_workspace_guard")
    cartesian_workspace_radius_m = LaunchConfiguration("cartesian_workspace_radius_m")
    cartesian_workspace_base_link = LaunchConfiguration("cartesian_workspace_base_link")
    cartesian_workspace_tip_link = LaunchConfiguration("cartesian_workspace_tip_link")
    cartesian_workspace_reset_on_streaming_activate = LaunchConfiguration(
        "cartesian_workspace_reset_on_streaming_activate"
    )
    cartesian_workspace_reset_on_arm = LaunchConfiguration("cartesian_workspace_reset_on_arm")
    auto_disarm_on_workspace_violation = LaunchConfiguration("auto_disarm_on_workspace_violation")

    bridge = Node(
        package="abb_pi0_bridge",
        executable="abb_pi0_bridge_node",
        name="abb_pi0_bridge",
        output="screen",
        parameters=[
            bridge_params_file,
            {
                "publish_commands": publish_commands,
                "control_mode": control_mode,
                "policy_server_url": policy_server_url,
                "policy_request_timeout_sec": policy_request_timeout_sec,
                "enable_front_camera_observation": enable_front_camera_observation,
                "enable_wrist_camera_observation": enable_wrist_camera_observation,
                "front_camera_image_topic": front_camera_image_topic,
                "wrist_camera_image_topic": wrist_camera_image_topic,
                "camera_image_timeout_sec": camera_image_timeout_sec,
                "max_camera_joint_skew_sec": max_camera_joint_skew_sec,
                "auto_disarm_on_observation_fault": auto_disarm_on_observation_fault,
                "enable_cartesian_workspace_guard": enable_cartesian_workspace_guard,
                "cartesian_workspace_radius_m": cartesian_workspace_radius_m,
                "cartesian_workspace_base_link": cartesian_workspace_base_link,
                "cartesian_workspace_tip_link": cartesian_workspace_tip_link,
                "cartesian_workspace_reset_on_streaming_activate": cartesian_workspace_reset_on_streaming_activate,
                "cartesian_workspace_reset_on_arm": cartesian_workspace_reset_on_arm,
                "auto_disarm_on_workspace_violation": auto_disarm_on_workspace_violation,
            },
        ],
    )

    policy_stub = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [FindPackageShare("abb_pi0_bridge"), "..", "..", "lib", "abb_pi0_bridge", "pi0_policy_stub_server"]
            ),
            "--host",
            policy_server_host,
            "--port",
            policy_server_port,
            "--mode",
            policy_stub_mode,
            "--step-rad",
            policy_stub_step_rad,
            "--joint-index",
            policy_stub_joint_index,
        ],
        output="screen",
        condition=IfCondition(launch_policy_stub),
    )

    actions = [
            SetEnvironmentVariable("HTTP_PROXY", ""),
            SetEnvironmentVariable("http_proxy", ""),
            SetEnvironmentVariable("HTTPS_PROXY", ""),
            SetEnvironmentVariable("https_proxy", ""),
            SetEnvironmentVariable("ALL_PROXY", ""),
            SetEnvironmentVariable("all_proxy", ""),
            DeclareLaunchArgument(
                "robotstudio_rws_ip",
                default_value="192.168.125.1",
                description="RobotStudio/controller RWS IP reachable from this server. This real ABB workcell's verified controller address is 192.168.125.1.",
            ),
            DeclareLaunchArgument(
                "robotstudio_rws_port",
                default_value="80",
                description="RobotStudio virtual controller RWS TCP port reachable from this server.",
            ),
            DeclareLaunchArgument(
                "robotstudio_rws_username",
                default_value="Default User",
                description="RobotStudio virtual controller RWS username.",
            ),
            DeclareLaunchArgument(
                "robotstudio_rws_password",
                default_value="robotics",
                description="RobotStudio virtual controller RWS password.",
            ),
            DeclareLaunchArgument(
                "robot_controller_id",
                default_value="IRB6700",
                description="Controller id prefix used by abb_hardware_interface when parsing RWS metadata.",
            ),
            DeclareLaunchArgument(
                "egm_port",
                default_value="6515",
                description="EGM UDP port expected by the ABB driver and RobotStudio VC.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Whether to launch RViz with MoveIt.",
            ),
            DeclareLaunchArgument(
                "launch_moveit",
                default_value="false",
                description="Whether to launch MoveIt beside the RobotStudio bridge.",
            ),
            DeclareLaunchArgument(
                "launch_click_to_move",
                default_value="false",
                description="Whether to start the click_to_move node beside the bridge.",
            ),
            DeclareLaunchArgument(
                "initial_joint_controller",
                default_value="forward_command_controller_position",
                description="Initial ros2_control controller for RobotStudio mode. Use forward_command_controller_position for streaming, or joint_trajectory_controller for MoveIt-oriented bringup.",
            ),
            DeclareLaunchArgument(
                "launch_policy_stub",
                default_value="true",
                description="Whether to start the local HTTP policy stub server.",
            ),
            DeclareLaunchArgument(
                "policy_server_host",
                default_value="127.0.0.1",
                description="Bind host for the local policy stub server.",
            ),
            DeclareLaunchArgument(
                "policy_server_port",
                default_value="8000",
                description="Bind port for the local policy stub server.",
            ),
            DeclareLaunchArgument(
                "policy_stub_mode",
                default_value="hold",
                description="Mode for the local policy stub server: hold or left.",
            ),
            DeclareLaunchArgument(
                "policy_stub_step_rad",
                default_value="0.1",
                description="Joint step used by policy_stub_mode=left.",
            ),
            DeclareLaunchArgument(
                "policy_stub_joint_index",
                default_value="0",
                description="Zero-based joint index used by policy_stub_mode=left.",
            ),
            DeclareLaunchArgument(
                "policy_server_url",
                default_value="http://127.0.0.1:8000/infer",
                description="HTTP endpoint for the bridge policy backend.",
            ),
            DeclareLaunchArgument(
                "bridge_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("abb_pi0_bridge"), "config", "pi0_robotstudio_http.params.yaml"]
                ),
                description="Parameter file for abb_pi0_bridge in RobotStudio mode.",
            ),
            DeclareLaunchArgument(
                "publish_commands",
                default_value="false",
                description="Whether to allow filtered low-level streaming commands to leave the bridge.",
            ),
            DeclareLaunchArgument(
                "control_mode",
                default_value="trajectory",
                description="Initial bridge mode: trajectory, streaming, or monitor.",
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
                default_value="5.0",
                description="HTTP policy request timeout in seconds.",
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
                "enable_cartesian_workspace_guard",
                default_value="false",
                description="Reject bridge commands whose TCP target leaves the configured Cartesian radius. Defaults false here because RobotStudio-only launches may not pass robot_description into the bridge.",
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
            bridge,
            policy_stub,
        ]

    if _package_exists("abb_bringup"):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("abb_bringup"), "launch", "abb_control.launch.py"]
                    )
                ),
                launch_arguments={
                    "runtime_config_package": "workcell_config",
                    "description_package": "workcell_description",
                    "description_file": "workcell.urdf.xacro",
                    "use_fake_hardware": "false",
                    "rws_ip": robotstudio_rws_ip,
                    "rws_port": robotstudio_rws_port,
                    "rws_username": robotstudio_rws_username,
                    "rws_password": robotstudio_rws_password,
                    "robot_controller_id": robot_controller_id,
                    "egm_port": egm_port,
                    "initial_joint_controller": initial_joint_controller,
                    "launch_rviz": "false",
                }.items(),
            )
        )
    else:
        actions.append(
            LogInfo(
                msg="RobotStudio launch: package 'abb_bringup' not found, so ABB ros2_control bringup is skipped in this environment."
            )
        )

    moveit_stack_available = _package_exists("abb_bringup") and _package_exists("click_to_move") and _module_exists(
        "moveit_configs_utils"
    )

    if moveit_stack_available:
        from moveit_configs_utils import MoveItConfigsBuilder

        moveit_config = MoveItConfigsBuilder(
            "workcell", package_name="workcell_config"
        ).to_moveit_configs()

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("abb_bringup"), "launch", "abb_moveit.launch.py"]
                    )
                ),
                condition=IfCondition(launch_moveit),
                launch_arguments={
                    "robot_xacro_file": "workcell.urdf.xacro",
                    "support_package": "workcell_description",
                    "moveit_config_package": "workcell_config",
                    "moveit_config_file": "workcell.srdf",
                    "use_fake_hardware": "false",
                    "launch_rviz": launch_rviz,
                }.items(),
            )
        )

        actions.append(
            Node(
                package="click_to_move",
                executable="click_to_move_node",
                name="click_to_move",
                output="screen",
                condition=IfCondition(launch_click_to_move),
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
        )
    else:
        actions.append(
            LogInfo(
                msg="RobotStudio launch: MoveIt/click_to_move is unavailable here because 'moveit_configs_utils' or required packages are missing; bridge-only mode will still start."
            )
        )

    return LaunchDescription(actions)
