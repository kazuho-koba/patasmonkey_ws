"""実センサ・local EKF・現行認識・非駆動MPPI・設定GUIをまとめて起動する。

drive=false: ODriveはIDLEで状態取得のみ。drive=true: Joyによる既存手動操縦を起動。
どちらもexecution_managerは起動しないため、このツールのMPPIは実車を駆動しない。
具体的な通常センサ・localization構成は ../pm_bringup/stack_launch.py を再利用する。
"""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config=Path(get_package_share_directory('pm_config'))/'config'
    bringup=Path(get_package_share_directory('pm_bringup'))/'launch'
    perception=Path(get_package_share_directory('pm_perception'))
    control=yaml.safe_load((config/'vehicle_control.yaml').read_text())['vehicle_interface_node']['ros__parameters']
    # 独立observerへ必要な軸・符号だけを渡す。制御用の速度・gain設定は渡さない。
    observer_parameters={key:control[key] for key in ('mtr_axis_l','mtr_axis_r','left_motor_sign','right_motor_sign')}
    panel=Node(package='pm_gui',executable='perception_lab',name='perception_lab_panel',
               parameters=[{'drive_enabled':ParameterValue(LaunchConfiguration('drive'),value_type=bool)}],
               output='screen',condition=IfCondition(LaunchConfiguration('gui')))
    stack=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup/'pm_core.launch.py')),
        launch_arguments=dict(use_gnss='false',use_ntrip='false',use_joy='true',use_teleop='false',
            use_vehicle_interface='false',use_execution_manager='false',use_mission_receiver='false',
            # OpenVINSだけは下の専用Nodeで停止初期化を明示。adapter・gateはCoreを再利用。
            use_openvins='false',record_bag='false',localization_mode='separated_offroad',
            mapper_executable='planner_mapper_node',
            mapper_config_file=LaunchConfiguration('params_file')).items())
    return LaunchDescription([
        DeclareLaunchArgument('drive',default_value='false',description='trueだけJoy手動操縦を有効化。plannerは常に表示専用'),
        DeclareLaunchArgument('start_stack',default_value='true',description='falseは外部のセンサ・TF・認識mapを表示する試験用'),
        DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('params_file',default_value=str(config/'planner_perception.yaml')),
        DeclareLaunchArgument('vio_config_file',default_value=str(config/'oak_d_s2/estimator_config1.yaml')),
        GroupAction(condition=IfCondition(LaunchConfiguration('start_stack')),actions=[
            # Includeは引数を同じcontextへ設定するため、CoreとDriveを別scopeにする。
            # Drive側のuse_joy=falseがCoreの遅延起動へ漏れることを防ぐ。
            GroupAction(scoped=True,actions=[stack]),
            Node(package='pm_control',executable='motor_state_observer',output='screen',
                 parameters=[observer_parameters],condition=UnlessCondition(LaunchConfiguration('drive'))),
            # 既存pm_driveは中立確認を要求する。Joy driverはCore側だけで起動する。
            GroupAction(scoped=True,condition=IfCondition(LaunchConfiguration('drive')),actions=[
                IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup/'pm_drive.launch.py')))]),
            Node(package='ov_msckf',executable='run_subscribe_msckf',namespace='ov_msckf',output='screen',
                 parameters=[{'config_path':LaunchConfiguration('vio_config_file'),
                    'try_zupt':True,'verbosity':'WARNING','imu_history_depth':32,
                    'imu_input_diagnostics':False,'save_total_state':False,
                    'record_timing_information':False}]),
        ]),
        Node(package='pm_planner',executable='perception_preview',output='screen'),
        panel,
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(perception/'rviz/perception_lab.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        RegisterEventHandler(OnProcessExit(target_action=panel,on_exit=[
            EmitEvent(event=Shutdown(reason='環境認識パネルを終了しました'))])),
    ])
