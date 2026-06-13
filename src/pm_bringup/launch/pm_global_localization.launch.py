#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction


def generate_launch_description():
    # GNSS情報をどの程度使うか決めるパラメータ
    use_gnss = LaunchConfiguration("use_gnss")
    use_ntrip = LaunchConfiguration("use_ntrip")
    # Visual Odometryを使うかどうかのパラメータ
    use_oakd = LaunchConfiguration("use_oakd")
    use_openvins = LaunchConfiguration("use_openvins")
    
    # 各種パッケージのパス
    pm_teleop_share = Path(get_package_share_directory("pm_teleop"))
    pm_vehicle_share = Path(
        get_package_share_directory("pm_vehicle_interface"))
    pm_description_share = Path(get_package_share_directory("pm_description"))
    pm_config_share = Path(get_package_share_directory("pm_config"))
    depthai_driver_share = Path(get_package_share_directory("depthai_driver"))
    ov_msckf_share = Path(get_package_share_directory("ov_msckf"))

    # 既存launchファイル
    teleop_launch_file = pm_teleop_share/"launch"/"joy_teleop.launch.py"
    vehicle_launch_file = pm_vehicle_share/"launch"/"vehicle_interface.launch.py"
    openvins_launch_file = ov_msckf_share/"launch"/"subscribe.launch.py"

    # configファイル等
    urdf_file = pm_description_share/"urdf"/"pm.urdf"
    imu_config_file = pm_config_share/"config"/"hwt905_imu.yaml"
    wheel_odom_config_file = pm_config_share / "config" / "wheel_odometry.yaml"
    vehicle_geometry_file = pm_config_share / "config" / "vehicle_geometry.yaml"
    vehicle_control_file = pm_config_share / "config" / "vehicle_control.yaml"

    ekf_local_config_file = pm_config_share/"config"/"ekf_local_3d.yaml"
    ekf_global_config_file = pm_config_share/"config"/"ekf_global_3d.yaml"
    navsat_config_file = pm_config_share / "config" / "navsat_transform.yaml"

    ublox_config_file = pm_config_share / "config" / "ublox_f9p.yaml"
    ntrip_config_file = pm_config_share / "config" / "ntrip_local.yaml"

    openvins_config_file = pm_config_share / "config" / "oak_d_s2" / "estimator_config1.yaml"

    with open(urdf_file, "r") as f:
        robot_description = f.read()

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(teleop_launch_file)))
    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(vehicle_launch_file)))
    openvins_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(openvins_launch_file)),
        launch_arguments={
            "config_path": str(openvins_config_file),
            "rviz_enable": "false",
            "verbosity": "INFO",
        }.items(),
        condition=IfCondition(use_openvins),
    )
        

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, }],
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )
    imu_node = Node(
        package="hwt905_rs485_driver",
        executable="hwt905_imu_node",
        name="hwt905_imu_node",
        output="screen",
        parameters=[str(imu_config_file)],
    )
    wheel_odometry_node = Node(
        package="pm_localization",
        executable="wheel_odometry_node",
        name="wheel_odometry_node",
        output="screen",
        parameters=[
            str(vehicle_geometry_file),
            str(vehicle_control_file),
            str(wheel_odom_config_file),
        ],
    )
    oakd_vio_rgbd_node = Node(
        package="depthai_driver",
        executable="oakd_vio_rgbd_node",
        name="oakd_vio_rgbd_node",
        output="screen",
        condition=IfCondition(use_oakd),
    )
    # odom_to_path_node = Node(
    #     package="pm_localization",
    #     executable="odom_to_path_node",
    #     name="odom_to_path_node",
    #     output="screen",
    #     parameters=[
    #         str(pm_config_share/"config"/"wheel_odometry.yaml"),
    #     ],
    # )
    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[str(ekf_local_config_file)],
    )

    ublox_gps_node = Node(
        package="ublox_gps",
        executable="ublox_gps_node",
        name="ublox_gps_node",
        output="screen",
        parameters=[str(ublox_config_file)],
        condition=IfCondition(use_gnss),
        respawn=True,
        respawn_delay=3.0,
    )

    ntrip_client_node = Node(
        package="ntrip_client",
        executable="ntrip_client_node",
        name="ntrip_client_node",
        output="screen",
        parameters=[str(ntrip_config_file)],
        condition=IfCondition(use_ntrip),
    )

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[str(navsat_config_file)],
        remappings=[
            ("imu/data", "/imu/data"),
            ("gps/fix", "/fix"),
            ("odometry/filtered", "/odometry/local"),
            ("odometry/gps", "/odometry/gps"),
        ],
        condition=IfCondition(use_gnss),
    )

    ekf_global_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global_node",
        output="screen",
        parameters=[str(ekf_global_config_file)],
        condition=IfCondition(use_gnss),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gnss",
            default_value="true",
            description="Start GNSS, navsat_transform, and global EKF",
        ),
        DeclareLaunchArgument(
            "use_ntrip",
            default_value="true",
            description="Start NTRIP client for RTK corrections",
        ),
        DeclareLaunchArgument(
            "use_oakd",
            default_value="true",
            description="Start OAK-D S2 RGB-D/VIO sensor node",
        ),
        DeclareLaunchArgument(
            "use_openvins",
            default_value="true",
            description="Start OpenVINS",
        ),

        teleop_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,

        # GNSSを最優先で起動
        ublox_gps_node,

        # 3秒後: IMU/local odometry系
        TimerAction(
            period=3.0,
            actions=[
                imu_node,
                wheel_odometry_node,
                ekf_local_node,
            ],
        ),

        # 6秒後: NTRIP開始
        TimerAction(
            period=3.0,
            actions=[
                ntrip_client_node,
            ],
        ),

        # 9秒後: GNSS変換/global EKF開始
        TimerAction(
            period=9.0,
            actions=[
                navsat_transform_node,
                ekf_global_node,
            ],
        ),

        # 12秒後: ODrive
        TimerAction(
            period=12.0,
            actions=[
                vehicle_launch,
            ],
        ),

        # 15秒後: OAK-D/OpenVINS
        TimerAction(
            period=15.0,
            actions=[
                oakd_vio_rgbd_node,
                openvins_launch,
            ],
        ),
    ])
