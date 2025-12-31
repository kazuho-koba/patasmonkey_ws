from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="patasmonkey_vehicle_interface",
            executable="dummy_jointstate_pub",
            name="dummy_jointstate_pub",
            parameters=[
                {"left_cycle_per_sec":1.0},
                {"right_cycle_per_sec":1.0},
            ],
            output="screen",
        ),
        Node(
            package="patasmonkey_vehicle_interface",
            executable="wheel_odometry_node",
            name="wheel_odometry_node",
            parameters=[
                "src/patasmonkey_vehicle_interface/config/vehicle_params.yaml"
            ],
            output="screen",
        ),
    ])