"""５方式と履歴parameterから、実機を含まない再計算graphを生成する。"""
import math
from pathlib import Path
import yaml

from pm_evaluation.localization_batch_io import MODES

MODE_FILES = {
    'legacy': ('ekf_local_whl_imu_cam.yaml', 'ekf_local_node'),
    'separated_offroad': ('ekf_local_horizontal_vio_twist.yaml', 'ekf_local_horizontal_node'),
    'separated_projected': ('ekf_local_horizontal_projected.yaml', 'ekf_local_horizontal_node'),
    'ekf_3d_no_accel': ('ekf_local_3d_no_accel.yaml', 'ekf_local_3d_node'),
    'ekf_3d_accel': ('ekf_local_3d_accel.yaml', 'ekf_local_3d_node'),
}


def config_values(config, filename, node):
    return yaml.safe_load((config/filename).read_text())[node]['ros__parameters']


def clean(values):
    """Foxy snapshotに現れる未設定nullはparameter overrideへ渡さない。"""
    return {k:v for k,v in values.items() if v is not None and v != [] and not k.startswith('qos_overrides.')}


def create_job(bag, output, config, mode, yaw, snapshot, historical=False,
               wheel_only=False, duration=None, rate=1.):
    """製品と同じ推定ノードを接続。初期yawはENUとしてinitial_stateへ１回設定。"""
    output.mkdir(parents=True, exist_ok=True)
    nodes=[]; used=[]; overrides=[]
    captured=snapshot.get('snapshots', {})

    def values(filename, original):
        result=config_values(config,filename,original)
        if historical and '/'+original in captured:
            result.update(clean(captured['/'+original]['values']))
            used.append('/'+original)
        elif historical:
            overrides.append(original+': 保存値なし。現行設定で代用')
        return result

    def add(package, executable, name, params, remaps=()):
        params.update(use_sim_time=True)
        file=name+'.yaml'
        (output/file).write_text(yaml.safe_dump({name:{'ros__parameters':clean(params)}},sort_keys=False))
        nodes.append(dict(package=package,executable=executable,name=name,
                          parameters=file,remaps=list(remaps)))

    # 3D共分散は保持し、2D EKF専用の周辺観測だけを別topicへ作る。
    add('pm_localization','planar_twist_covariance_node','batch_planar_covariance',{})
    file,original=MODE_FILES[mode]
    local=values(file,original)
    if local.get('two_d_mode'):
        if 'odom1' in local:local['odom1']='/vio/odometry/twist_planar'
    local.update(publish_tf=True, publish_acceleration=True,
                 initial_state=[0.,0.,0.,0.,0.,yaw]+[0.]*9)
    if wheel_only:
        # VIOの観測vectorを無効化。topicを別名にするだけで旧履歴が混入しない。
        for key in list(local):
            if ((key.startswith('odom') and key!='odom0_config') or key=='twist1_config') and key.endswith('_config'):
                local[key]=[False]*15
    local_output='/odometry/local_horizontal' if mode in ('separated_offroad','separated_projected') else '/odometry/local'
    if local_output.endswith('_horizontal'):
        local['publish_tf']=False
    add('robot_localization','ekf_node','batch_local',local,
        [('odometry/filtered',local_output),('set_pose','/batch/local/set_pose'),
         ('accel/filtered','/batch/local/accel')])
    if mode=='separated_projected':
        add('pm_localization','horizontal_velocity_projection_node','batch_projection',{})
    if local_output.endswith('_horizontal'):
        add('pm_localization','attitude_height_observer_node','batch_vertical',{})
        add('pm_localization','local_odometry_composer_node','batch_local_composer',{})
    # 新adapterを実行し、raw ROS quaternion・取付位置・全共分散を一体変換する。
    if not wheel_only:
        add('pm_localization','vio_odom_adapter_node','batch_adapter',{
            'require_heading_ready':False, 'align_initial_to_tf':True,
            'initial_reference_topic':'/odometry/local'})
        for executable in ('vio_twist_gate_node','vio_vertical_gate_node'):
            params=clean(captured.get('/'+executable,{}).get('values',{})) if historical else {}
            add('pm_localization',executable,'batch_'+executable,params)
    global_=values('ekf_global_gnss_constrained.yaml','ekf_global_gnss_constrained_node')
    global_['odom1']='/vio/odometry/twist_planar'
    global_.update(two_d_mode=True,publish_tf=False,publish_acceleration=True,
                   initial_state=[0.,0.,0.,0.,0.,yaw]+[0.]*9)
    if wheel_only:
        global_['odom1_config']=[False]*15
    add('robot_localization','ekf_node','batch_global',global_,
        [('odometry/filtered','/odometry/global_horizontal'),
         ('set_pose','/batch/global/set_pose'),('accel/filtered','/batch/global/accel')])
    add('pm_localization','global_odometry_composer_node','batch_global_composer',{})
    add('pm_localization','gnss_fix_gate_node','batch_gnss_gate',
        values('gnss_fix_gate.yaml','gnss_fix_gate_node'))
    navsat=values('navsat_transform_heading_initialized.yaml','navsat_transform_node')
    # 過去bagでは静止初期化やdatumサービスの前提が欠け得るため、全条件共通の
    # 疑似初期化を使う。ENU yawを既にseedしたglobal odomを地球方位として渡す。
    # これは任意のGNSS軌跡fitではなく、navsat本来の初回変換初期化である。
    navsat.update(use_odometry_yaw=True,wait_for_datum=False,yaw_offset=0.,
                  magnetic_declination_radians=0.,broadcast_utm_transform=False)
    add('robot_localization','navsat_transform_node','batch_navsat',navsat,
        [('gps/fix','/fix/gated'),('odometry/filtered','/odometry/global'),
         ('odometry/gps','/odometry/gps')])
    topics=['/wheel/odometry','/wit/imu','/fix','/navpvt','/tf_static']
    if not wheel_only:topics.append('/ov_msckf/odomimu')
    outputs=['/odometry/local','/odometry/local_horizontal','/odometry/local_vertical',
             '/odometry/global','/odometry/global_horizontal','/odometry/gps','/tf',
             '/vio/odometry','/vio/odometry/twist_planar','/vio/odometry/twist_gated','/vio/odometry/gated',
             '/vio/adapter/diagnostics','/vio/twist_gate/diagnostics','/vio/vertical_gate/diagnostics',
             '/gnss/fix_gate/diagnostics','/batch/local/accel','/batch/global/accel']
    return dict(bag=str(bag),nodes=nodes,input_topics=topics,outputs=outputs,
                services=['/batch/local/set_pose','/batch/global/set_pose'],
                duration=duration,rate=rate,mode=mode,wheel_only=wheel_only,
                heading_yaw_rad=yaw,initialization='ENU yawのinitial_stateによる疑似初期化',
                historical_parameters=used,substitutions=overrides,
                limitations=['実行コードは現行。adapterは指定どおり新版',
                    'bag以前の状態・共分散は復元せず初期化',
                    'navsatは初期化済みENU odometry yawから自動datum。実機のサービス初期化は再現しない',
                    'globalの水平入力構成は全方式で共通、z/roll/pitchは各localから合成'])
