from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('workcell_description')
    xacro_path = PathJoinSubstitution([pkg, 'urdf', 'workcell.urdf.xacro'])

    prefix_arg  = DeclareLaunchArgument('prefix', default_value='robot1_')
    scale_v_arg = DeclareLaunchArgument('mesh_scale_visual', default_value='1 1 1')
    scale_c_arg = DeclareLaunchArgument('mesh_scale_collision', default_value='1 1 1')

    robot_description = {
        'robot_description': Command([
            'xacro ', xacro_path,
            ' ', 'prefix:=', LaunchConfiguration('prefix'),
            ' ', 'mesh_scale_visual:="', LaunchConfiguration('mesh_scale_visual'), '"',
            ' ', 'mesh_scale_collision:="', LaunchConfiguration('mesh_scale_collision'), '"'
        ])
    }

    return LaunchDescription([
        prefix_arg, scale_v_arg, scale_c_arg,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_description], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2')
    ])
