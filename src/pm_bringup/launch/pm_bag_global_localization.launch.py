#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # GNSS情報をどの程度使うか決めるパラメータ
    use_gnss = LaunchConfiguration("use_gnss")
    use_ntrip = LaunchConfiguration("use_ntrip")
    # Visual Odometryを使うかどうかのパラメータ
    use_oakd = LaunchConfiguration("use_oakd")
    use_openvins = LaunchConfiguration("use_openvins")
    # rosbagを記録するかどうか
    record_bag = LaunchConfiguration("record_bag")
    
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

    ekf_local_config_file = pm_config_share/"config"/"ekf_local_whl_imu_cam.yaml"
    ekf_global_config_file = pm_config_share/"config"/"ekf_global_3d.yaml"
    navsat_config_file = pm_config_share / "config" / "navsat_transform.yaml"

    ublox_config_file = pm_config_share / "config" / "ublox_f9p.yaml"
    ntrip_config_file = pm_config_share / "config" / "ntrip_private.yaml"

    openvins_config_file = pm_config_share / "config" / "oak_d_s2" / "estimator_config1.yaml"

    # rosbag保存先
    bag_output_directory = Path.home() / "patasmonkey_ws" / "bags"
    bag_output_directory.mkdir(parents=True, exist_ok=True)
    # rosbagに記録するトピック
    #
    # 方針:
    # - VO、wheel odometry、EKF、GNSSをオフライン再計算できる入力を保存
    # - オンライン計算結果も比較用に保存
    # - RGB、Depthを認識・自律走行研究用に保存
    # - OpenVINSの画像transport派生トピックやGT系は保存しない
    bag_topics = [
        # -------------------------------------------------------------
        # OAK-D：OpenVINS再計算用
        # -------------------------------------------------------------
        "/oak/stereo/left/image_raw",
        "/oak/stereo/right/image_raw",
        "/oak/imu/data",

        # -------------------------------------------------------------
        # OAK-D：RGB-D認識、オフロード走行解析用
        # -------------------------------------------------------------
        "/oak/color/image_raw",
        "/oak/depth/image_raw",

        # -------------------------------------------------------------
        # 外部IMU・磁気センサ
        # -------------------------------------------------------------
        "/wit/imu",
        "/wit/mag",
        "/imu",

        # -------------------------------------------------------------
        # 車両状態・wheel odometry
        # -------------------------------------------------------------
        "/motor_state",
        "/wheel/odometry",

        # -------------------------------------------------------------
        # OpenVINS・VIOオンライン出力
        # -------------------------------------------------------------
        "/ov_msckf/odomimu",
        "/ov_msckf/poseimu",
        "/ov_msckf/pathimu",
        "/vio/odometry",
        # VOの作動状況を一応記録
        "/ov_msckf/trackhist",

        # -------------------------------------------------------------
        # robot_localization出力
        # -------------------------------------------------------------
        "/odometry/local",
        "/odometry/global",
        "/odometry/gps",
        "/gps/filtered",

        # -------------------------------------------------------------
        # GNSS測位結果
        # -------------------------------------------------------------
        "/fix",
        "/fix_velocity",
        "/navpvt",
        "/navrelposned",
        "/navheading",
        "/navstatus",
        "/navstate",
        "/navclock",
        "/navsvin",

        # GNSS受信機・ハードウェア診断
        "/monhw",

        # -------------------------------------------------------------
        # RTK補正情報
        # -------------------------------------------------------------
        # NTRIP clientが受信し、F9Pへ渡すRTCM
        "/rtcm",

        # F9Pが実際に受信・解析したRTCM情報
        "/rxmrtcm",

        # -------------------------------------------------------------
        # 車両への指令・操作履歴
        # -------------------------------------------------------------
        "/pm/joy",
        "/cmd_vel_joy",
        "/cmd_vel",
        "/sim_cmd_vel",
        "/emergency_stop",

        # -------------------------------------------------------------
        # TF・RobotModel・RViz再現用
        # -------------------------------------------------------------
        "/tf",
        "/tf_static",
        "/robot_description",
        "/joint_states",

        # -------------------------------------------------------------
        # 診断・実験状態
        # -------------------------------------------------------------
        "/diagnostics",
        "/rosout",
        "/parameter_events",
        "/set_pose",
    ]

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

    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[str(ekf_local_config_file)],
        remappings=[
            ("odometry/filtered", "/odometry/local"),
        ],
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
            ("imu/data", "/wit/imu"),
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
        remappings=[
            ("odometry/filtered", "/odometry/global"),
        ],
        condition=IfCondition(use_gnss),
    )

    rosbag_record_process = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-s",
            "mcap",
            "--max-bag-size",
            "1000000000",
            *bag_topics,
        ],
        cwd=str(bag_output_directory),
        output="screen",
        condition=IfCondition(record_bag),
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
            "record_bag",
            default_value="true",
            description="Record all ROS 2 topics to an MCAP rosbag",
        ),

        # 実際のコマンド実行・ノード起動など
        rosbag_record_process,

        teleop_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,

        # GNSSを最優先で起動
        ublox_gps_node,

        # 3秒後: IMU/local odometry系とrtk信号送受信ノード立ち上げ
        TimerAction(
            period=3.0,
            actions=[
                imu_node,
                wheel_odometry_node,
                ekf_local_node,
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
            ],
        ),

        # GNSS座標変換とglobal EKFだけ遅らせる
        TimerAction(
            period=15.0,
            actions=[
                navsat_transform_node,
                ekf_global_node,
            ],
        ),
    ])
