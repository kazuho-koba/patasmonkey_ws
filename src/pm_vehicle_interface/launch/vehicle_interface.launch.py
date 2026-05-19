from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.abspath(__file__))  # package directory
    

    pm_config_dir = Path(get_package_share_directory("pm_config"))
    vehicle_geometry_yaml = pm_config_dir / "config" / "vehicle_geometry.yaml"
    vehicle_control_yaml = pm_config_dir / "config" / "vehicle_control.yaml"

    # Define the vehicle interface node
    vehicle_interface_node = Node(
        package="pm_vehicle_interface",
        executable="vehicle_interface_node",
        name="vehicle_interface_node",
        parameters=[
            str(vehicle_geometry_yaml),
            str(vehicle_control_yaml),
        ],
        output="screen",
    )

    # Ensure clean shutdown on ROS2 shutdown
    shutdown_handler = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
            ],
        )
    )

    return LaunchDescription([
        vehicle_interface_node,
        shutdown_handler,  # Ensure proper cleanup when the node exits
    ])
