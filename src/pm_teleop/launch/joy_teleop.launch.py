from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_teleop = get_package_share_directory('pm_teleop')
    # get abs path for config files
    joy_params_path = os.path.join(pkg_teleop, 'config', 'joy_params.yaml')
    teleop_twist_joy_params_path = os.path.join(pkg_teleop, 'config', 'teleop_twist_joy.yaml')

    # joy入力監視と速度指令変換を別々に起動できる。単体launchの既定値は維持する。
    return LaunchDescription([
        DeclareLaunchArgument("use_joy", default_value="true"),
        DeclareLaunchArgument("use_teleop", default_value="true"),

        # joy_node (get command info from your game pad)
        Node(
            package='joy',  # joy package: standard node for joysticks in ROS2
            executable='joy_node',
            name='joy_node',
            condition=IfCondition(LaunchConfiguration('use_joy')),
            parameters=[joy_params_path],  # read param file
            remappings=[('/joy', '/pm/joy')],  # remap the topic name
            output='screen'
        ),

        # teleop_twist_joy (convert game pad's input to velocity command)
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='joy_teleop',
            condition=IfCondition(LaunchConfiguration('use_teleop')),
            parameters=[teleop_twist_joy_params_path],
            remappings=[('/joy', '/pm/joy'),
                        ('/cmd_vel', '/cmd_vel_joy')],
            output='screen'
        ),
    ])
