"""機器や走行制御を起動せず、現行の分離local/global推定器をbagで再生する。"""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('pm_config'))/'config'
    urdf = Path(get_package_share_directory('pm_description'))/'urdf'/'pm.urdf'
    with (config/'hwt905_mag_calibration.yaml').open(encoding='utf-8') as stream:
        mag = yaml.safe_load(stream)['wit_mag_corrector']['ros__parameters']
    # Foxyではノード名付きYAMLが汎用の後続パラメータより優先されるため、
    # 試験設定を同じ辞書にまとめてから適用し、TFの二重発行を防ぐ。
    with (config/'ekf_global_gnss_constrained.yaml').open(encoding='utf-8') as stream:
        global_horizontal = yaml.safe_load(stream)['ekf_global_gnss_constrained_node']['ros__parameters']
    global_horizontal.update({'two_d_mode': True, 'publish_tf': False})
    # 過去条件との比較用。現行製品は水平合成方式を使う。
    global_3d = dict(global_horizontal)
    global_3d.update({'two_d_mode': False, 'publish_tf': True,
                      'imu0_config': [False, False, False, True, True, False,
                                      False, False, False, False, False, True,
                                      False, False, False]})
    sim = {'use_sim_time': True}
    nodes = [
        # 比較試験でnavsatの基準odomだけを変えられる。既定は本番設定と同じ。
        DeclareLaunchArgument('navsat_odom_topic',default_value='/odometry/global'),
        DeclareLaunchArgument('global_separated',default_value='true'),
        # 独立した評価用YAMLでlocal EKFの前提だけを切り替える。
        DeclareLaunchArgument('local_ekf_config',default_value=str(config/'ekf_local_horizontal_vio_twist.yaml')),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim, {'robot_description': urdf.read_text(encoding='utf-8')}]),
        Node(package='hwt905_rs485_driver', executable='wit_mag_corrector',
             parameters=[str(config/'hwt905_mag_calibration.yaml'), sim]),
        Node(package='pm_localization', executable='vio_twist_gate_node', parameters=[sim]),
        Node(package='pm_localization', executable='vio_vertical_gate_node', parameters=[sim]),
        Node(package='pm_localization', executable='gnss_fix_gate_node',
             parameters=[str(config/'gnss_fix_gate.yaml'), sim]),
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_local_horizontal_node',
             parameters=[LaunchConfiguration('local_ekf_config'), sim],
             remappings=[('odometry/filtered','/odometry/local_horizontal'),
                         ('set_pose','/localization/local/set_pose')]),
        Node(package='pm_localization', executable='attitude_height_observer_node', parameters=[sim]),
        Node(package='pm_localization', executable='local_odometry_composer_node', parameters=[sim]),
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_global_gnss_constrained_node',
             condition=UnlessCondition(LaunchConfiguration('global_separated')),
             parameters=[global_3d, sim],
             remappings=[('odometry/filtered','/odometry/global'),
                         ('set_pose','/localization/global/set_pose')]),
        # 本番と同じ方式：global水平EKFへlocalの姿勢・相対zを合成。
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_global_gnss_constrained_node',
             condition=IfCondition(LaunchConfiguration('global_separated')),
             parameters=[global_horizontal, sim],
             remappings=[('odometry/filtered','/odometry/global_horizontal'),
                         ('set_pose','/localization/global/set_pose')]),
        Node(package='pm_evaluation', executable='global_separated_composer',
             condition=IfCondition(LaunchConfiguration('global_separated')),parameters=[sim]),
        Node(package='robot_localization', executable='navsat_transform_node',
             name='navsat_transform_node',
             parameters=[str(config/'navsat_transform_heading_initialized.yaml'), sim],
             remappings=[('gps/fix','/fix/gated'),
                         ('odometry/filtered',LaunchConfiguration('navsat_odom_topic')),
                         ('odometry/gps','/odometry/gps/unverified')]),
        Node(package='pm_localization', executable='heading_initializer_node',
             parameters=[str(config/'heading_initializer.yaml'), sim, {
                 'offset_deg': float(mag['offset_deg']),
                 'declination_deg': float(mag['declination_deg']), 'require_gnss': True}]),
    ]
    return LaunchDescription(nodes)
