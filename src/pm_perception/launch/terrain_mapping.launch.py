from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("pm_perception"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("camera_frame_override", default_value=""),
            Node(
                package="pm_perception",
                executable="depth_elevation_mapper_node",
                name="depth_elevation_mapper",
                output="screen",
                parameters=[
                    str(share / "config" / "depth_elevation_mapper.yaml"),
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        # An empty override uses the Image header. Passing the
                        # historical alias makes old bags match their URDF TF.
                        "camera_frame_override": LaunchConfiguration(
                            "camera_frame_override"
                        ),
                    },
                ],
            ),
        ]
    )
