#!/usr/bin/env python3

from datetime import datetime
from pathlib import Path
import json

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
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def create_launch_description(legacy_recording=False, drive_default=True):
    """共通UGV起動構成。旧launchだけは従来の一括記録も維持する。

    新pm_coreではrecorderを起動せず、OpenVINS等の付帯ログをROS home側へ分離する。
    drive_default=Falseでは入力監視だけ残し、走行は独立launchで明示的に許可する。
    旧launchの制御既定値はdrive_default=Trueで維持する。
    """
    # GNSS情報をどの程度使うか決めるパラメータ
    use_gnss = LaunchConfiguration("use_gnss")
    use_ntrip = LaunchConfiguration("use_ntrip")
    # Visual Odometryを使うかどうかのパラメータ
    use_oakd = LaunchConfiguration("use_oakd")
    use_openvins = LaunchConfiguration("use_openvins")
    use_joy = LaunchConfiguration("use_joy")
    use_teleop = LaunchConfiguration("use_teleop")
    use_vehicle_interface = LaunchConfiguration("use_vehicle_interface")
    mapper_callback_diagnostics = LaunchConfiguration(
        "mapper_callback_diagnostics"
    )
    mapper_depth_subscription_queue_depth = LaunchConfiguration(
        "mapper_depth_subscription_queue_depth"
    )
    mapper_publish_stage2_debug_layers = LaunchConfiguration(
        "mapper_publish_stage2_debug_layers"
    )
    mapper_tf_retry_rate_hz = LaunchConfiguration("mapper_tf_retry_rate_hz")
    mapper_debug_publish_rate = LaunchConfiguration("mapper_debug_publish_rate")
    mapper_executor_diagnostics = LaunchConfiguration(
        "mapper_executor_diagnostics"
    )
    mapper_executor_diagnostics_csv = LaunchConfiguration(
        "mapper_executor_diagnostics_csv"
    )
    mapper_tf_listener_dedicated_thread = LaunchConfiguration(
        "mapper_tf_listener_dedicated_thread"
    )
    # rosbagを記録するかどうか
    record_bag = LaunchConfiguration("record_bag")
    bag_name = LaunchConfiguration("bag_name")
    # legacy local EKFを既定とする。off-road向けにlocalizerを分離する場合は
    # `localization_mode:=separated_offroad`を選ぶ。
    local_ekf_config = LaunchConfiguration("local_ekf_config")
    localization_mode = LaunchConfiguration("localization_mode")

    # 各種パッケージのパス
    pm_teleop_share = Path(get_package_share_directory("pm_teleop"))
    pm_vehicle_share = Path(
        get_package_share_directory("pm_control"))
    pm_description_share = Path(get_package_share_directory("pm_description"))
    pm_config_share = Path(get_package_share_directory("pm_config"))
    pm_perception_share = Path(get_package_share_directory("pm_perception"))
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
    terrain_mapper_config_file = (
        pm_perception_share / "config" / "depth_elevation_mapper.yaml"
    )

    # rosbag保存先
    bag_output_directory = (Path.home() / "patasmonkey_ws" / "bags" if legacy_recording
                            else Path.home() / ".ros" / "pm_core_sessions")
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
        """node起動前にbag出力先とlegacy EKF設定を検証する."""
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

        # legacy modeではCLIで指定された設定ファイル名を実パスへ解決する。
        # YAMLが存在しないままEKFを無設定で起動する事態を、他nodeの起動前に防ぐ。
        if localization_mode.perform(context) == "legacy":
            resolved_ekf_config = Path(ekf_local_config_file.perform(context))
            if not resolved_ekf_config.is_file():
                raise RuntimeError(
                    "Legacy EKF config does not exist: {}".format(
                        resolved_ekf_config
                    )
                )
        if not legacy_recording:
            # 実効launch引数を永続nodeのparameterとして公開する。独立起動した
            # mission recorderが同じROS graphから取得し、bagへ同梱できる。
            from pm_bringup.recording_metadata import redact
            manifest = {
                "launch": "pm_core.launch.py",
                "arguments": redact(dict(context.launch_configurations)),
                "runtime_directory": str(resolved_directory),
                "configuration_paths": [str(path) for path in (
                    urdf_file, imu_config_file, wheel_odom_config_file,
                    vehicle_geometry_file, vehicle_control_file, openvins_config_file,
                    ekf_global_config_file, ekf_horizontal_config_file,
                    heading_initializer_config_file, gnss_fix_gate_config_file,
                    navsat_config_file, navsat_heading_initialized_config_file,
                    pm_perception_share / 'config' / 'depth_elevation_mapper.yaml',
                )] + [ekf_local_config_file.perform(context)],
            }
            (resolved_directory / 'openvins').mkdir(parents=True)
            runtime_actions = [Node(
                package="pm_bringup", executable="core_manifest",
                name="pm_core_manifest", output="screen",
                parameters=[{"manifest_json": ParameterValue(
                    json.dumps(manifest, ensure_ascii=False, default=str), value_type=str)}],
            )] + runtime_actions
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
        # OAK-D：RGB-D認識用画像と各画像geometryを再現するCameraInfo。
        # -------------------------------------------------------------
        "/oak/color/image_raw",
        "/oak/color/camera_info",
        "/oak/depth/image_raw",
        "/oak/depth/camera_info",
        "/oak/stereo/confidence/image_raw",
        "/oak/stereo/disparity/image_raw",
        "/oak/stereo/recording_snapshot",
        "/oak/diagnostics/confidence_frame",
        "/oak/diagnostics/disparity_frame",

        # -------------------------------------------------------------
        # Stage 2 local terrain mappingの出力。上記のdepth input、CameraInfo、現在の
        # localisation output、TFだけでmapperをreplayできる。加えてこれらのlayerを保存し、
        # 再構成なしにonline結果を直接比較できるようにする。
        # -------------------------------------------------------------
        "/depth_elevation_mapper/elevation_debug",
        "/depth_elevation_mapper/relative_elevation_debug",
        "/depth_elevation_mapper/elevation_variance_debug",
        "/depth_elevation_mapper/observation_count_debug",
        "/depth_elevation_mapper/observation_age_debug",
        "/depth_elevation_mapper/obstacle_height_debug",
        # Stage 3 feature cueはplanner costではなくdiagnosticとして記録する。
        "/depth_elevation_mapper/slope_debug",
        "/depth_elevation_mapper/roughness_debug",
        "/depth_elevation_mapper/step_height_debug",
        "/depth_elevation_mapper/terrain_hazard_debug",
        "/depth_elevation_mapper/terrain_hazard_cause_debug",

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
        # intermediate outputを保存するとseparated profileを検証可能になる。horizontal
        # VIO/wheelの問題とattitude/heightの問題を区別できる。
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
        PythonLaunchDescriptionSource(str(teleop_launch_file)),
        # 入力監視は走行許可と独立。速度指令の変換だけをOFFにできる。
        launch_arguments={"use_joy": use_joy, "use_teleop": use_teleop}.items(),
    )
    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(vehicle_launch_file)),
        condition=IfCondition(use_vehicle_interface),
    )
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
        # 診断を有効化した試行だけ、終了時に同じbagディレクトリへCSVを保存する。
        # poll_hzや通信設定は既存configのまま保持する。
        parameters=[str(imu_config_file), {
            "timing_diagnostics": LaunchConfiguration("wit_timing_diagnostics"),
            "timing_csv": PathJoinSubstitution([
                str(bag_output_directory), bag_name, "wit_timing.csv",
            ]),
            "timing_max_samples": LaunchConfiguration("wit_timing_max_samples"),
        }],
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
        parameters=[{
            'publish_depth_confidence': ParameterValue(
                LaunchConfiguration('oak_publish_depth_confidence'), value_type=bool),
            'confidence_threshold': ParameterValue(
                LaunchConfiguration('oak_confidence_threshold'), value_type=int),
        }],
    )
    terrain_mapper_node = Node(
        package="pm_perception",
        executable="depth_elevation_mapper_node",
        name="depth_elevation_mapper",
        output="screen",
        parameters=[
            str(terrain_mapper_config_file),
            {
                # YAMLと同じwildcard scopeで試験overrideを後から渡し、Foxyで既定値を
                # queue深度・診断parameterが上書きできるようにする。
                "diagnostic_callback_timing": ParameterValue(
                    mapper_callback_diagnostics, value_type=bool
                ),
                # QoS履歴深度だけを変える試験用override。既定値5では従来どおり。
                "depth_subscription_queue_depth": ParameterValue(
                    mapper_depth_subscription_queue_depth, value_type=int
                ),
                "depth_subscription_reliability": ParameterValue(
                    LaunchConfiguration("mapper_depth_subscription_reliability"), value_type=str
                ),
                # Stage 2 layerの出力だけを個別比較し、hazard計算は維持する。
                "publish_stage2_debug_layers": ParameterValue(
                    mapper_publish_stage2_debug_layers, value_type=bool
                ),
                # TF待ちqueueの再試行周期だけを独立して変更する。
                "tf_retry_rate_hz": ParameterValue(
                    mapper_tf_retry_rate_hz, value_type=float
                ),
                # Stage 2出力比較ではhazard評価周期を2 Hzに固定できる。
                "debug_publish_rate": ParameterValue(
                    mapper_debug_publish_rate, value_type=float
                ),
                # callbackごとの計測を行うFoxy executor wrapper。通常は無効。
                "diagnostic_executor_timing": ParameterValue(
                    mapper_executor_diagnostics, value_type=bool
                ),
                "diagnostic_executor_csv_path": ParameterValue(
                    mapper_executor_diagnostics_csv, value_type=str
                ),
                "tf_listener_dedicated_thread": ParameterValue(
                    mapper_tf_listener_dedicated_thread, value_type=bool
                ),
            },
        ],
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

            # このadapterはOdometryだけをpublishする。`odom -> base_link` TFは
            # robot_localizationがpublishする。
            'publish_tf': False,
        }]
    )

    # off-road local-EKF profileがVIO z poseを使う前にgateする。正常dataはそのまま通し、
    # VIO reset/divergence後は閉状態をlatchする。legacy EKF profileは引き続き
    # `/vio/odometry`を直接使う。
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

    # `local_ekf_config`はlaunch引数から実行時に選ばれるため、
    # PathJoinSubstitutionを文字列化せずNodeへ渡す。
    # `str()`にすると解決済みパスではなくPython objectのreprになり、
    # EKFがYAMLを読まず、設定topicやodom TFを生成できなくなる。
    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[ekf_local_config_file],
        condition=IfCondition(PythonExpression([
            "'", localization_mode, "' == 'legacy'",
        ])),
        remappings=[
            ("odometry/filtered", "/odometry/local"),
        ],
    )
    # このprofileではVIO poseをhorizontal x/y/yawから分離する。
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
    # stationary中にyawを一度seedするのはseparated profileだけである。また
    # navsat_transformが使うcalibrated heading streamもpublishする。
    heading_initializer_node = Node(
        package="pm_localization", executable="heading_initializer_node",
        name="heading_initializer_node", output="screen",
        parameters=[str(heading_initializer_config_file)],
        condition=IfCondition(PythonExpression([
            "'", localization_mode, "' == 'separated_offroad'",
        ])),
    )
    # datum初期化とnavsat_transformの両方より前にraw GNSSをgateする。raw `/fix`と
    # `/navpvt`はpost-run audit用に利用可能なまま記録する。
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
            # robot_localization 3.1.x（ROS 2 Foxy）は`imu/data`ではなくrelative name
            # `imu`をsubscribeする。後者をremapするとnavsat_transformへheading dataが
            # 届かず、`/odometry/gps`がpublishされない。
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
            # heading initializerはroll/pitchとIMU rateを変えず、検証済みのyaw補正だけを
            # orientationへ適用する。
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
    # separated profileはraw wheel/IMU/VIO velocityから直接predictし、`/odometry/gps`を
    # 唯一のmap-position observationとする。
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

    runtime_actions = ([rosbag_record_process, TimerAction(
            period=2.0,
            actions=[capture_vio_metadata_process],
        )] if legacy_recording else []) + [
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
                terrain_mapper_node,
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
        DeclareLaunchArgument('oak_publish_depth_confidence', default_value='true',
                              description='同sequenceのconfidence/disparityと校正設定を追加出力'),
        DeclareLaunchArgument('oak_confidence_threshold', default_value='240',
                              description='StereoDepth生成時のconfidence閾値0..255（小ほど厳格）'),
        DeclareLaunchArgument(
            "wit_timing_diagnostics", default_value="false",
            description="Witの読み取り・publish・周期遅延を終了時にCSV保存する",
        ),
        DeclareLaunchArgument(
            "wit_timing_max_samples", default_value="60000",
            description="Wit周期診断の保持件数。超過時は古い記録を破棄する",
        ),
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
        DeclareLaunchArgument("use_joy", default_value="true",
                              description="操縦入力を監視するjoy nodeを起動する"),
        DeclareLaunchArgument(
            "use_teleop",
            default_value="true" if drive_default else "false",
            description="ジョイスティック操縦を起動する。センサ単独試験ではfalseにする",
        ),
        DeclareLaunchArgument(
            "use_vehicle_interface",
            default_value="true" if drive_default else "false",
            description="モーター制御可能な車両interfaceを起動する。センサ単独試験ではfalseにする",
        ),
        DeclareLaunchArgument(
            "mapper_callback_diagnostics",
            default_value="false",
            description=(
                "mapperのdepth/TF callback間隔・age・処理時間をログする。"
                "負荷測定時のみtrueにする"
            ),
        ),
        DeclareLaunchArgument(
            "mapper_depth_subscription_queue_depth",
            default_value="3",
            description=(
                "mapperのdepth subscriber KEEP_LAST履歴深度。実機比較後の暫定既定3。"
                "TF待ちpending queueとは独立"
            ),
        ),
        DeclareLaunchArgument(
            "mapper_depth_subscription_reliability", default_value="best_effort",
            description="depth受信の比較用QoS: best_effort / reliable。既定は従来どおり",
        ),
        DeclareLaunchArgument(
            "mapper_publish_stage2_debug_layers",
            default_value="true",
            description=(
                "relative/variance/count/age/obstacleの5 debug layer出力。"
                "比較時はこの値だけをfalseへ切り替える"
            ),
        ),
        DeclareLaunchArgument(
            "mapper_tf_retry_rate_hz",
            default_value="100.0",
            description=(
                "撮像timestampのTF待ちqueueを再確認する周期。"
                "A/B比較値は100/30/20 Hz"
            ),
        ),
        DeclareLaunchArgument(
            "mapper_debug_publish_rate",
            default_value="2.0",
            description=(
                "hazard評価・debug snapshotの周期。Stage 2出力比較では2 Hz固定"
            ),
        ),
        DeclareLaunchArgument(
            "mapper_executor_diagnostics",
            default_value="false",
            description=(
                "mapper SingleThreadedExecutorのready entity、dispatch gap、"
                "callback wall/CPU時間をCSV記録する診断専用override"
            ),
        ),
        DeclareLaunchArgument(
            "mapper_executor_diagnostics_csv",
            default_value="",
            description=(
                "executor診断CSVの出力先。空ならmapperが一意な/tmp名を作る"
            ),
        ),
        DeclareLaunchArgument(
            "record_bag",
            default_value="true" if legacy_recording else "false",
            description="旧一括launch用。pm_coreでは記録せずmission/debug launchで記録する",
        ),
        DeclareLaunchArgument(
            "mapper_tf_listener_dedicated_thread",
            default_value="false",
            description="TF受信だけを専用node/threadへ分離する比較用設定。撮像時刻TFは維持",
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
