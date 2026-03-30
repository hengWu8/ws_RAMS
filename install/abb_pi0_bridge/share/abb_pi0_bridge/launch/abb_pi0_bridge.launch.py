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
                    },
                ],
            ),
        ]
    )
