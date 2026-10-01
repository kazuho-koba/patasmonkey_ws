#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
    # Keep the legacy local EKF as the default. Select localization_mode:=
    # separated_offroad for the off-road-safe, split localizer.
    local_ekf_config = LaunchConfiguration("local_ekf_config")
    localization_mode = LaunchConfiguration("localization_mode")
    
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

    ekf_local_config_file = PathJoinSubstitution([
        str(pm_config_share), "config", local_ekf_config,
    ])
    ekf_global_config_file = pm_config_share/"config"/"ekf_global_3d.yaml"
    ekf_global_gnss_constrained_config_file = pm_config_share / "config" / "ekf_global_gnss_constrained.yaml"
    ekf_horizontal_config_file = pm_config_share / "config" / "ekf_local_horizontal_vio_twist.yaml"
    heading_initializer_config_file = pm_config_share / "config" / "heading_initializer.yaml"
    gnss_fix_gate_config_file = pm_config_share / "config" / "gnss_fix_gate.yaml"
    navsat_config_file = pm_config_share / "config" / "navsat_transform.yaml"
    navsat_heading_initialized_config_file = pm_config_share / "config" / "navsat_transform_heading_initialized.yaml"

    ublox_config_file = pm_config_share / "config" / "ublox_f9p.yaml"
    ntrip_config_file = pm_config_share / "config" / "ntrip_private.yaml"

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
            "verbosity": "WARNING",
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
    vio_odom_adapter_node = Node(
        package='pm_localization',
        executable='vio_odom_adapter_node',
        name='vio_odom_adapter_node',
        output='screen',
        parameters=[{
            'input_topic': '/ov_msckf/odomimu',
            'output_topic': '/vio/odometry',

            'output_frame_id': 'odom',
            'output_child_frame_id': 'base_link',

            'base_frame_id': 'base_link',
            'oak_imu_frame_id': 'openvins_imu_link',

            'invert_openvins_orientation': True,
            'align_initial_to_tf': False,
            'zero_initial_pose': True,

            'align_output_orientation_to_initial_tf': True,
            'invert_relative_rotation': True,   

            # 診断用パラメータ
            'enable_diagnostics': True,
            'diagnostics_interval_sec': 0.1,

            # This adapter publishes Odometry only.
            # Let robot_localization publish odom -> base_link TF.
            'publish_tf': False,
        }]
    )

    # Used only by the separated_offroad profile. The legacy profile keeps
    # subscribing to /vio/odometry directly and is intentionally unchanged.
    vio_vertical_gate_node = Node(
        package="pm_localization",
        executable="vio_vertical_gate_node",
        name="vio_vertical_gate_node",
        output="screen",
    )
    vio_twist_gate_node = Node(
        package="pm_localization", executable="vio_twist_gate_node",
        name="vio_twist_gate_node", output="screen",
    )
    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[str(ekf_local_config_file)],
        condition=IfCondition(PythonExpression([
            "'", localization_mode, "' == 'legacy'",
        ])),
        remappings=[
            ("odometry/filtered", "/odometry/local"),
        ],
    )
    ekf_local_horizontal_node = Node(
        package="robot_localization", executable="ekf_node",
        name="ekf_local_horizontal_node", output="screen",
        parameters=[str(ekf_horizontal_config_file)],
        condition=IfCondition(PythonExpression([
            "'", localization_mode, "' == 'separated_offroad'",
        ])),
        remappings=[("odometry/filtered", "/odometry/local_horizontal")],
    )
    attitude_height_observer_node = Node(
        package="pm_localization", executable="attitude_height_observer_node",
        name="attitude_height_observer_node", output="screen",
        condition=IfCondition(PythonExpression([
            "'", localization_mode, "' == 'separated_offroad'",
        ])),
    )
    local_odometry_composer_node = Node(
        package="pm_localization", executable="local_odometry_composer_node",
        name="local_odometry_composer_node", output="screen",
        condition=IfCondition(PythonExpression([
            "'", localization_mode, "' == 'separated_offroad'",
        ])),
    )
    heading_initializer_node = Node(
        package="pm_localization", executable="heading_initializer_node",
        name="heading_initializer_node", output="screen",
        parameters=[str(heading_initializer_config_file)],
        condition=IfCondition(PythonExpression([
            "'", localization_mode, "' == 'separated_offroad'",
        ])),
    )
    gnss_fix_gate_node = Node(
        package="pm_localization", executable="gnss_fix_gate_node",
        name="gnss_fix_gate_node", output="screen",
        parameters=[str(gnss_fix_gate_config_file)],
        condition=IfCondition(PythonExpression([
            "'", use_gnss, "' == 'true' and '", localization_mode,
            "' == 'separated_offroad'",
        ])),
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

    navsat_transform_legacy_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[str(navsat_config_file)],
        remappings=[
            # Foxy robot_localization navsat_transform_node subscribes to
            # "imu" (not "imu/data").
            ("imu", "/wit/imu"),
            ("gps/fix", "/fix"),
            ("odometry/filtered", "/odometry/local"),
            ("odometry/gps", "/odometry/gps"),
        ],
        condition=IfCondition(PythonExpression([
            "'", use_gnss, "' == 'true' and '", localization_mode,
            "' == 'legacy'",
        ])),
    )
    navsat_transform_separated_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[str(navsat_heading_initialized_config_file)],
        remappings=[
            ("imu", "/wit/imu/heading_calibrated"),
            ("gps/fix", "/fix/gated"),
            ("odometry/filtered", "/odometry/local"),
            ("odometry/gps", "/odometry/gps"),
        ],
        condition=IfCondition(PythonExpression([
            "'", use_gnss, "' == 'true' and '", localization_mode,
            "' == 'separated_offroad'",
        ])),
    )

    ekf_global_legacy_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global_node",
        output="screen",
        parameters=[str(ekf_global_config_file)],
        remappings=[
            ("odometry/filtered", "/odometry/global"),
        ],
        condition=IfCondition(PythonExpression([
            "'", use_gnss, "' == 'true' and '", localization_mode,
            "' == 'legacy'",
        ])),
    )
    ekf_global_gnss_constrained_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global_gnss_constrained_node",
        output="screen",
        parameters=[str(ekf_global_gnss_constrained_config_file)],
        remappings=[("odometry/filtered", "/odometry/global")],
        condition=IfCondition(PythonExpression([
            "'", use_gnss, "' == 'true' and '", localization_mode,
            "' == 'separated_offroad'",
        ])),
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
        DeclareLaunchArgument(
            "local_ekf_config",
            default_value="ekf_local_whl_imu_cam.yaml",
            description=(
                "Filename under pm_config/config for the legacy one-EKF mode. "
                "For the off-road separated profile, select "
                "localization_mode:=separated_offroad instead."
            ),
        ),
        DeclareLaunchArgument(
            "localization_mode",
            default_value="legacy",
            description=(
                "legacy: one EKF; separated_offroad: horizontal EKF plus a "
                "validated attitude/height observer, composed into /odometry/local"
            ),
        ),

        teleop_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,

        # GNSSを最優先で起動
        ublox_gps_node,
        gnss_fix_gate_node,

        # 3秒後: IMU/local odometry系とrtk信号送受信ノード立ち上げ
        TimerAction(
            period=3.0,
            actions=[
                imu_node,
                wheel_odometry_node,
                ekf_local_node,
                ekf_local_horizontal_node,
                attitude_height_observer_node,
                local_odometry_composer_node,
                heading_initializer_node,
                ntrip_client_node,
            ],
        ),

        # 6秒後: ODrive, 外界センサ系
        TimerAction(
            period=6.0,
            actions=[
                vehicle_launch,
                oakd_vio_rgbd_node,
                openvins_launch,
                vio_odom_adapter_node,
                vio_vertical_gate_node,
                vio_twist_gate_node,
            ],
        ),

        # GNSS座標変換とglobal EKFだけ遅らせる
        TimerAction(
            period=30.0,
            actions=[
                navsat_transform_legacy_node,
                navsat_transform_separated_node,
                ekf_global_legacy_node,
                ekf_global_gnss_constrained_node,
            ],
        ),
    ])
