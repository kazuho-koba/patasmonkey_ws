from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("pm_perception"))
    return LaunchDescription(
        [
            Node(
                package="pm_perception",
                executable="depth_elevation_mapper_node",
                name="depth_elevation_mapper",
                output="screen",
                parameters=[
                    str(share / "config" / "depth_elevation_mapper.yaml"),
                    str(share / "config" / "depth_elevation_mapper_old_bag.yaml"),
                ],
            )
        ]
    )
