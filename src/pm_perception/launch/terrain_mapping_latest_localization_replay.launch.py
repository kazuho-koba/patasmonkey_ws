"""Replay a recorded RGB-D bag through the current separated localizer.

The companion bag_clock_player must publish only sensor inputs, /tf_static,
and images. It intentionally must not replay the bag's historical /tf: this
launch owns the replacement odom -> base_link transform.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    perception_share = Path(get_package_share_directory("pm_perception"))
    config_share = Path(get_package_share_directory("pm_config"))
    description_share = Path(get_package_share_directory("pm_description"))
    horizontal_config = config_share / "config" / "ekf_local_horizontal_vio_twist.yaml"
    mapper_config = perception_share / "config" / "depth_elevation_mapper.yaml"
    old_bag_config = perception_share / "config" / "depth_elevation_mapper_old_bag.yaml"
    with open(description_share / "urdf" / "pm.urdf", "r") as urdf_stream:
        robot_description = urdf_stream.read()

    sim_time = {"use_sim_time": True}
    return LaunchDescription(
        [
            # The bag supplies its recorded /tf_static while the current
            # localizer supplies odom -> base_link.  Publish this replay-only
            # robot model's fixed transforms privately so it can provide
            # /robot_description to RViz without competing with the bag TF.
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="terrain_replay_joint_state_publisher",
                output="screen",
                parameters=[sim_time],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="terrain_replay_robot_state_publisher",
                output="screen",
                parameters=[sim_time, {"robot_description": robot_description}],
                remappings=[("/tf_static", "/terrain_replay/robot_model_tf_static")],
            ),
            Node(
                package="pm_localization",
                executable="vio_vertical_gate_node",
                name="vio_vertical_gate_node",
                output="screen",
                parameters=[sim_time],
            ),
            Node(
                package="pm_localization",
                executable="vio_twist_gate_node",
                name="vio_twist_gate_node",
                output="screen",
                parameters=[sim_time],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                # The YAML is scoped to this node name. Keep it identical to
                # the runtime separated_offroad launch so its parameters are
                # actually loaded during replay.
                name="ekf_local_horizontal_node",
                output="screen",
                parameters=[str(horizontal_config), sim_time],
                remappings=[("odometry/filtered", "/odometry/local_horizontal")],
            ),
            Node(
                package="pm_localization",
                executable="attitude_height_observer_node",
                name="terrain_replay_attitude_height_observer",
                output="screen",
                parameters=[
                    sim_time,
                    {"output_topic": "/odometry/local_vertical"},
                ],
            ),
            Node(
                package="pm_localization",
                executable="local_odometry_composer_node",
                name="terrain_replay_local_odometry_composer",
                output="screen",
                parameters=[
                    sim_time,
                    {
                        "horizontal_topic": "/odometry/local_horizontal",
                        "vertical_topic": "/odometry/local_vertical",
                        "output_topic": "/odometry/local",
                        "publish_tf": True,
                    },
                ],
            ),
            Node(
                package="pm_perception",
                executable="depth_elevation_mapper_node",
                name="depth_elevation_mapper",
                output="screen",
                parameters=[str(mapper_config), str(old_bag_config)],
            ),
        ]
    )
