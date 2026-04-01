from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    rws_ip = DeclareLaunchArgument(
        "rws_ip",
        default_value="192.168.1.1",
        description="RWS IP reachable from the ROS2 machine.",
    )
    rws_port = DeclareLaunchArgument(
        "rws_port",
        default_value="80",
        description="RWS TCP port reachable from the ROS2 machine.",
    )
    egm_port = DeclareLaunchArgument(
        "egm_port",
        default_value="6515",
        description="EGM UDP port exposed by the ABB controller or RobotStudio VC.",
    )

    fixed_serial = DeclareLaunchArgument(
        "fixed_serial", default_value="_109522062115",  # 注意：去掉了你命令里的下划线 "_"
        description="Serial number of the fixed RealSense camera"
    )
    wrist_serial = DeclareLaunchArgument(
        "wrist_serial", default_value="_342522070232",  # 注意：去掉了你命令里的下划线 "_"
        description="Serial number of the wrist RealSense camera"
    )

    fixed_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={
            "camera_name": "fixed_cam",
            "serial_no": LaunchConfiguration("fixed_serial"),
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
            "align_depth": "true",
            "pointcloud.enable": "true",
            "color_qos": "SENSOR_DATA",
            "depth_qos": "SENSOR_DATA",
        }.items(),
    )

    # ========== 外参（静态 TF）==========
    # base_link -> fixed_cam_link
    tf_fixed = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_fixed_cam",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "fixed_cam_link"],
        output="screen",
    )

    # tool0 -> wrist_cam_link
    tf_wrist = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_wrist_cam",
        arguments=["0", "0", "0", "0", "0", "0", "tool0", "wrist_cam_link"],
        output="screen",
    )
    # # world -> base_link（很多时候由 bringup 带起；这里加上更稳）
    # tf_world_base = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="tf_world_base",
    #     arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    #     output="screen",
    # )

    # ========== ABB 控制 & MoveIt ==========
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
            "launch_rviz": "false",
        }.items(),
    )

    abb_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("abb_bringup"), "launch", "abb_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "robot_xacro_file": "workcell.urdf.xacro",
            "support_package": "workcell_description",
            "moveit_config_package": "workcell_config",
            "moveit_config_file": "workcell.srdf",
            "use_fake_hardware": "false",
            "launch_rviz": "true",
        }.items(),
    )

    moveit_config = MoveItConfigsBuilder(
        "workcell", package_name="workcell_config"
    ).to_moveit_configs()

    click_to_move = Node(
        package="click_to_move",
        executable="click_to_move_node",
        name="click_to_move",
        output="screen",
        parameters=[
            moveit_config.to_dict(),  # 关键：注入 robot_description / SRDF / kinematics
            {
                "planning_group": "abb_irb6700",
                "base_frame": "base",
                "eef_link": "",  # 走默认末端
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

    return LaunchDescription([
        rws_ip,
        rws_port,
        egm_port,
        fixed_serial,
        wrist_serial,
        fixed_cam,
        wrist_cam,
        tf_fixed,
        tf_wrist,
        # tf_world_base,
        abb_control, abb_moveit,
        click_to_move,
    ])
