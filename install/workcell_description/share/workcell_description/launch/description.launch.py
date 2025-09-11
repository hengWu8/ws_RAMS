from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg = get_package_share_directory('workcell_description')
    xacro_file = os.path.join(pkg, 'urdf', 'workcell.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': False,
                         'robot_description': robot_description_config}],
            output='screen'
        ),
        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui')
    ])