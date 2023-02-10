from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

"""" 
    This launch file:
    - Specifies the path to a params file
    - Declares a joy_node node that uses the params file & launches it
    - Declares a teleop_node node that uses the params file & launches it
"""

def generate_launch_description():

    joy_params = os.path.join(
            get_package_share_directory('manolobot_uno'),
            'config',
            'joystick.yaml'
        )

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params],
         )

    teleop_node = Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name = 'teleop_node',
            parameters=[joy_params],
            remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')]
        )

    return LaunchDescription([
        joy_node,
        teleop_node,
    ])