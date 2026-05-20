from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pm_config_share = get_package_share_directory("pm_config")

    ekf_config = os.path.join(
        pm_config_share,
        'config',
        'ekf_wheel_imu.yaml'
    )

    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_config],
        ),
    ])
