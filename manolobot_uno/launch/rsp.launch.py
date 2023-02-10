import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    package_name = 'manolobot_uno'

    # Check parameters that toggle use of sim time and ros2_control
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Process the URDF file
    xacro_file_path = os.path.join(get_package_share_directory(package_name),'description','robot.urdf.xacro')

    robot_description_config = Command(['xacro ', xacro_file_path, ' use_ros2_control:=',use_ros2_control])

    # Create a robot_state_publisher node
    params = {
        'robot_description': robot_description_config, 
        'use_sim_time': use_sim_time,
    }
    node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params]
        )

    # Launch
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),
        node_robot_state_publisher
    ])
