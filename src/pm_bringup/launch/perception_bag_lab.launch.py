"""bagからlocal推定・現行認識・非駆動MPPI・GUI/RVizを起動する。

オンラインCore・Driveはincludeしない。旧動的TF・速度指令・既存costは再生せず、
bagのwheel/Wit/VIOからEKFと撮像時刻TFを再生成する。
"""
from pathlib import Path
import yaml
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent, LogInfo
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pm_evaluation.perception_bag_player import bag_topics,replay_topics


def setup(context):
    """bagの実在topicを検査してから、重複しない再計算用graphを組む。"""
    bag=Path(LaunchConfiguration('bag').perform(context)).expanduser().resolve()
    rate=float(LaunchConfiguration('rate').perform(context))
    duration=float(LaunchConfiguration('duration_sec').perform(context))
    if not math.isfinite(rate) or not .1<=rate<=4.:raise ValueError('rateは0.1〜4で指定してください')
    if not math.isfinite(duration) or duration<0:raise ValueError('duration_secは0以上で指定してください')
    if LaunchConfiguration('reference_mode').perform(context) not in ('fixed_start','current_pose'):
        raise ValueError('reference_modeはfixed_start/current_pose')
    available=bag_topics(bag)
    topics,raw=replay_topics(available,LaunchConfiguration('vio_source').perform(context))
    config=Path(get_package_share_directory('pm_config'))/'config'
    perception=Path(get_package_share_directory('pm_perception'))
    description=Path(get_package_share_directory('pm_description'))
    old=LaunchConfiguration('old_bag').perform(context).lower()=='true'
    if '/oak/depth/camera_info' not in available and not old:
        raise RuntimeError('depth CameraInfoがありません。校正の一致を確認した旧OAK bagはold_bag:=trueを指定してください')
    sim={'use_sim_time':True}
    overrides=dict(sim)
    if old:
        # 7月OAK bagと実機EEPROMの照合済み値を既存設定から再利用する。
        # 別個体へ自動的に適合する校正ではない。old_bagは明示指定だけで有効化する。
        previous=yaml.safe_load((perception/'config/depth_elevation_mapper.yaml').read_text())['/**']['ros__parameters']
        overrides.update({name:previous[name] for name in ('fallback_fx','fallback_fy','fallback_cx',
            'fallback_cy','fallback_width','fallback_height')})
        overrides.update(allow_fallback_intrinsics=True,camera_frame_override='rgb_camera_optical_frame')
    params=LaunchConfiguration('params_file').perform(context)
    probe=LaunchConfiguration('planning_probe_dir').perform(context)
    if probe and Path(probe).exists():raise ValueError('planning_probe_dirには新しい出力ディレクトリを指定してください')
    preview_node=dict(package='pm_evaluation',executable='perception_preview_probe',arguments=['--output',probe]) if probe else dict(
        package='pm_planner',executable='perception_preview')
    panel=Node(package='pm_gui',executable='perception_lab',name='perception_lab_panel',output='screen',
        parameters=[dict(sim,drive_enabled=False,offline_mode=True)],
        condition=IfCondition(LaunchConfiguration('gui')))
    robot_tf=[('/tf_static','/perception_replay/model_tf_static')] if '/tf_static' in available else []
    actions=[LogInfo(msg='bag review: '+str(bag)+' / VIO='+('rawを最新adapterで変換' if raw else '記録済み/vio/odometry')),
        Node(package='robot_state_publisher',executable='robot_state_publisher',output='screen',
             parameters=[dict(sim,robot_description=(description/'urdf/pm.urdf').read_text())],remappings=robot_tf),
        Node(package='joint_state_publisher',executable='joint_state_publisher',parameters=[sim]),
        Node(package='robot_localization',executable='ekf_node',name='ekf_local_horizontal_node',output='screen',
            parameters=[str(config/'ekf_local_horizontal_vio_twist.yaml'),sim],
            remappings=[('odometry/filtered','/odometry/local_horizontal')]),
        Node(package='pm_localization',executable='vio_vertical_gate_node',parameters=[sim]),
        Node(package='pm_localization',executable='vio_twist_gate_node',parameters=[sim]),
        Node(package='pm_localization',executable='planar_twist_covariance_node',parameters=[sim]),
        Node(package='pm_localization',executable='attitude_height_observer_node',parameters=[sim]),
        Node(package='pm_localization',executable='local_odometry_composer_node',parameters=[sim]),
        Node(package='pm_perception',executable='planner_mapper_node',name='depth_elevation_mapper',
             output='screen',parameters=[params,overrides]),
        # bagのwheel/Witの初期座標で再計算するため、磁気方位初期化のREADYは要求しない。
        Node(**preview_node,output='screen',
             parameters=[dict(sim,require_heading_ready=False,reference_mode=LaunchConfiguration('reference_mode'))]),
        panel,
        Node(package='rviz2',executable='rviz2',parameters=[sim],
             arguments=['-d',str(perception/'rviz/perception_lab.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        RegisterEventHandler(OnProcessExit(target_action=panel,on_exit=[EmitEvent(event=Shutdown(reason='bag確認パネル終了'))])),
    ]
    if raw:
        actions.append(Node(package='pm_localization',executable='vio_odom_adapter_node',output='screen',
                            parameters=[dict(sim,require_heading_ready=False)]))
    # 最後にplayerを起動。自身はwall timerで進み、他の全nodeへ/clockを供給する。
    actions.append(Node(package='pm_evaluation',executable='perception_bag_player',output='screen',
        parameters=[{'bag':str(bag),'topics':topics,
                     'rate':rate,
                     'start_paused':LaunchConfiguration('start_paused').perform(context).lower()=='true',
                     'duration_sec':duration}]))
    return actions


def generate_launch_description():
    config=Path(get_package_share_directory('pm_config'))/'config'
    return LaunchDescription([
        DeclareLaunchArgument('bag',description='MCAP/SQLiteのbagディレクトリ'),
        DeclareLaunchArgument('rate',default_value='1.0'),
        DeclareLaunchArgument('start_paused',default_value='false'),
        DeclareLaunchArgument('duration_sec',default_value='0.0',description='bag秒での再生長。0は最後まで'),
        DeclareLaunchArgument('old_bag',default_value='false',description='照合済み旧OAK校正とoptical frame補正を使う'),
        DeclareLaunchArgument('vio_source',default_value='auto',description='auto/raw/recorded。autoはraw odomimu優先'),
        DeclareLaunchArgument('reference_mode',default_value='current_pose',description='current_pose:各再生地点からの直進 / fixed_start:初期直線を固定'),
        DeclareLaunchArgument('planning_probe_dir',default_value='',description='非空時だけ実計画入力を保存する診断nodeを使用。新しい出力ディレクトリを指定'),
        DeclareLaunchArgument('params_file',default_value=str(config/'planner_perception.yaml')),
        DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),
        OpaqueFunction(function=setup),
    ])
