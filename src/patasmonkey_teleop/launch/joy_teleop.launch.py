from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_teleop = get_package_share_directory('patasmonkey_teleop')
    # get abs path for config files
    joy_params_path = os.path.join(pkg_teleop, 'config', 'joy_params.yaml')
    teleop_twist_joy_params_path = os.path.join(pkg_teleop, 'config', 'teleop_twist_joy.yaml')

    return LaunchDescription([

        # joy_node (get command info from your game pad)
        Node(
            package='joy',  # joy package: standard node for joysticks in ROS2
            executable='joy_node',
            name='joy_node',
            parameters=[joy_params_path],  # read param file
            remappings=[('/joy', '/patasmonkey/joy')],  # remap the topic name
            output='screen'
        ),

        # teleop_twist_joy (convert game pad's input to velocity command)
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='joy_teleop',
            parameters=[teleop_twist_joy_params_path],
            remappings=[('/joy', '/patasmonkey/joy'),
                        ('/cmd_vel', '/cmd_vel_joy')],
            output='screen'
        ),
    ])
