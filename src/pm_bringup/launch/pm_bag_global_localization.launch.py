#!/usr/bin/env python3

from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
    bag_name = LaunchConfiguration("bag_name")
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
    ov_msckf_share = Path(get_package_share_directory("ov_msckf"))

    # 既存launchファイル
    teleop_launch_file = pm_teleop_share / "launch" / "joy_teleop.launch.py"
    vehicle_launch_file = (
        pm_vehicle_share / "launch" / "vehicle_interface.launch.py"
    )
    openvins_launch_file = ov_msckf_share / "launch" / "subscribe.launch.py"

    # configファイル等
    urdf_file = pm_description_share / "urdf" / "pm.urdf"
    imu_config_file = pm_config_share / "config" / "hwt905_imu.yaml"
    wheel_odom_config_file = pm_config_share / "config" / "wheel_odometry.yaml"
    vehicle_geometry_file = pm_config_share / "config" / "vehicle_geometry.yaml"
    vehicle_control_file = pm_config_share / "config" / "vehicle_control.yaml"

    ekf_local_config_file = PathJoinSubstitution([
        str(pm_config_share), "config", local_ekf_config,
    ])
    ekf_global_config_file = pm_config_share / "config" / "ekf_global_3d.yaml"
    ekf_global_gnss_constrained_config_file = (
        pm_config_share / "config" / "ekf_global_gnss_constrained.yaml"
    )
    ekf_horizontal_config_file = (
        pm_config_share / "config" / "ekf_local_horizontal_vio_twist.yaml"
    )
    heading_initializer_config_file = (
        pm_config_share / "config" / "heading_initializer.yaml"
    )
    gnss_fix_gate_config_file = pm_config_share / "config" / "gnss_fix_gate.yaml"
    navsat_config_file = pm_config_share / "config" / "navsat_transform.yaml"
    navsat_heading_initialized_config_file = (
        pm_config_share / "config" / "navsat_transform_heading_initialized.yaml"
    )

    ublox_config_file = pm_config_share / "config" / "ublox_f9p.yaml"
    ntrip_config_file = pm_config_share / "config" / "ntrip_private.yaml"

    openvins_config_file = (
        pm_config_share / "config" / "oak_d_s2" / "estimator_config1.yaml"
    )

    # rosbag保存先
    bag_output_directory = Path.home() / "patasmonkey_ws" / "bags"
    bag_output_directory.mkdir(parents=True, exist_ok=True)
    # bag名をlaunch側で決めることで、OpenVINS固有ログも同じ試行の
    # rosbagディレクトリ直下へ確実に関連付けて保存する。
    trial_directory = PathJoinSubstitution(
        [str(bag_output_directory), bag_name]
    )
    openvins_output_directory = PathJoinSubstitution(
        [trial_directory, "openvins"]
    )

    def validate_trial_directory(context, runtime_actions):
        """Reject ambiguous or existing destinations before starting nodes."""
        resolved_name = LaunchConfiguration("bag_name").perform(context)
        if not resolved_name or Path(resolved_name).name != resolved_name:
            raise RuntimeError(
                "bag_name must be one non-empty directory name: {}".format(
                    resolved_name
                )
            )
        resolved_directory = bag_output_directory / resolved_name
        if resolved_directory.exists():
            raise RuntimeError(
                "Refusing to overwrite existing trial directory: {}".format(
                    resolved_directory
                )
            )
        return runtime_actions

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
        # OAK-D：device時計・sequence・露光・IMU内部同期の診断
        # -------------------------------------------------------------
        # Image/Imu本体のheaderはOpenVINS互換のまま維持し、DepthAI固有の
        # monotonic timestampと欠落情報を構造化metadata topicに分離する。
        "/oak/diagnostics/left_frame",
        "/oak/diagnostics/right_frame",
        "/oak/diagnostics/color_frame",
        "/oak/diagnostics/depth_frame",
        "/oak/diagnostics/imu_packet",
        "/oak/diagnostics/device_info",

        # -------------------------------------------------------------
        # 外部IMU・磁気センサ
        # -------------------------------------------------------------
        "/wit/imu",
        "/wit/imu/heading_calibrated",
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
        "/vio/odometry/gated",
        "/vio/vertical_gate/diagnostics",
        "/vio/odometry/twist_gated",
        "/vio/twist_gate/diagnostics",

        # OpenVINSフロントエンド診断。
        # これらはsubscriberが存在するときだけOpenVINSが生成するため、
        # rosbag対象へ明示追加して特徴追跡・採択点を後から確認可能にする。
        "/ov_msckf/trackhist",
        "/ov_msckf/points_msckf",
        "/ov_msckf/points_slam",

        # -------------------------------------------------------------
        # robot_localization出力
        # -------------------------------------------------------------
        "/odometry/local",
        # Intermediate outputs make the separated profile auditable: these
        # distinguish a horizontal VIO/wheel issue from an attitude/height issue.
        "/odometry/local_horizontal",
        "/odometry/local_vertical",
        "/odometry/global",
        "/odometry/gps",
        "/gps/filtered",

        # -------------------------------------------------------------
        # GNSS測位結果
        # -------------------------------------------------------------
        "/fix",
        "/fix/gated",
        "/fix_velocity",
        "/navpvt",
        "/navrelposned",
        "/navheading",
        "/navstatus",
        "/navstate",
        "/navclock",
        "/navsvin",
        "/gnss/fix_gate/diagnostics",

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
            # DEBUGログには初期化判定など/rosoutへ出ない情報が含まれる。
            "verbosity": "DEBUG",
            "save_total_state": "true",
            "filepath_est": PathJoinSubstitution(
                [openvins_output_directory, "state_estimate.txt"]
            ),
            "filepath_std": PathJoinSubstitution(
                [openvins_output_directory, "state_deviation.txt"]
            ),
            "record_timing_information": "true",
            "record_timing_filepath": PathJoinSubstitution(
                [openvins_output_directory, "timing.txt"]
            ),
            "console_log_path": PathJoinSubstitution(
                [openvins_output_directory, "console.log"]
            ),
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

    # Gate VIO before the off-road local-EKF profile consumes its z pose.
    # It forwards healthy data unchanged and latches closed after a VIO reset
    # or divergence; legacy EKF profiles keep using /vio/odometry directly.
    vio_vertical_gate_node = Node(
        package="pm_localization",
        executable="vio_vertical_gate_node",
        name="vio_vertical_gate_node",
        output="screen",
    )
    vio_twist_gate_node = Node(
        package="pm_localization",
        executable="vio_twist_gate_node",
        name="vio_twist_gate_node",
        output="screen",
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
    # In this profile, VIO pose is isolated from horizontal x/y/yaw.
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
    # Only the separated profile seeds yaw once while stationary. It also
    # publishes the calibrated heading stream used by navsat_transform.
    heading_initializer_node = Node(
        package="pm_localization", executable="heading_initializer_node",
        name="heading_initializer_node", output="screen",
        parameters=[str(heading_initializer_config_file)],
        condition=IfCondition(PythonExpression([
            "'", localization_mode, "' == 'separated_offroad'",
        ])),
    )
    # Gate raw GNSS before both datum initialization and navsat_transform.
    # Raw /fix and /navpvt remain available and recorded for post-run audit.
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
            # robot_localization 3.1.x (ROS 2 Foxy) subscribes to the
            # relative name "imu", not "imu/data".  Remapping the latter
            # leaves navsat_transform without heading data and prevents
            # /odometry/gps from ever being published.
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
            # The heading initializer keeps roll/pitch and IMU rates unchanged,
            # but applies the one verified yaw correction to orientation.
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
    # The separated profile predicts directly from raw wheel/IMU/VIO velocity
    # and lets /odometry/gps be the sole map-position observation.
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

    rosbag_record_process = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            bag_name,
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

    # 使用したestimator/IMU/camera校正とGit revisionを同じ試行へ保存する。
    # rosbagが出力ディレクトリを作成した後、OpenVINS起動前に実行する。
    capture_vio_metadata_process = ExecuteProcess(
        cmd=[
            "ros2", "run", "pm_bringup", "capture_vio_trial_metadata",
            "--output-dir", openvins_output_directory,
            "--openvins-config", str(openvins_config_file),
            "--workspace", str(Path.home() / "patasmonkey_ws"),
            "--openvins-source", str(
                Path.home() / "ros2_ws" / "src" / "open_vins"
            ),
        ],
        output="screen",
    )

    # OpenVINS起動時にYAMLから上書きされた値も含め、実効ROSパラメータを
    # 保存する。OpenCV YAMLの全内容は上のconfig snapshotが担う。
    dump_openvins_parameters_process = ExecuteProcess(
        cmd=[
            "ros2", "param", "dump", "/ov_msckf/run_subscribe_msckf",
            "--output-dir", openvins_output_directory,
        ],
        output="screen",
        condition=IfCondition(use_openvins),
    )

    runtime_actions = [
        rosbag_record_process,
        TimerAction(
            period=2.0,
            actions=[capture_vio_metadata_process],
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
            period=15.0,
            actions=[
                navsat_transform_legacy_node,
                navsat_transform_separated_node,
                ekf_global_legacy_node,
                ekf_global_gnss_constrained_node,
            ],
        ),
        TimerAction(
            period=12.0,
            actions=[dump_openvins_parameters_process],
        ),
    ]

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
        DeclareLaunchArgument(
            "bag_name",
            default_value="rosbag2_" + datetime.now().strftime(
                "%Y_%m_%d-%H_%M_%S"
            ),
            description=(
                "Trial directory name under ~/patasmonkey_ws/bags; OpenVINS "
                "native diagnostics are stored in its openvins subdirectory"
            ),
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

        # 検証成功後にだけ全runtime actionを返すため、既存bagがある場合は
        # sensor/vehicle/OpenVINSのどのノードも起動しない。
        OpaqueFunction(
            function=validate_trial_directory,
            args=[runtime_actions],
        ),
    ])
