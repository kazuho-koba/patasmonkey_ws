from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
import os


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.abspath(__file__))  # package directory
    param_file = os.path.join(
        pkg_dir, "../config/vehicle_control_params.yaml")  # yaml file

    # Define the vehicle interface node
    vehicle_interface_node = Node(
        package="pm_vehicle_interface",
        executable="vehicle_interface_node",
        name="vehicle_interface_node",
        parameters=[param_file],
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
