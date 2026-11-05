"""記録済みのENU初期化を確認し、再計算開始時のlocal yawを取り出す。"""
import json
import math
import numpy as np
from pm_evaluation.localization_batch_io import header_sec
from pm_evaluation.localization_batch_plan import config_values
from pm_localization.vio_odom_adapter_node import quat_to_rot


def magnetic_seed_yaw(yaw_on, offset_deg, magnetic):
    """補正済みENU yaw[rad]から比較条件の初期yawを作る。

    実機はbearing = 磁気方位 + offset、yaw = π/2 - bearingなので、
    offsetを除去したyawはyaw_on + offsetとなる。負のoffsetを引くと
    逆方向へ補正してしまう。ONには既に補正が含まれるため追加しない。
    非有限値はEKF設定へ書き出す前に拒否する。
    """
    if magnetic not in ('on', 'off'):
        raise ValueError('磁気条件はon/offで指定')
    if not all(math.isfinite(v) for v in (yaw_on, offset_deg)):
        raise ValueError('初期yawまたは磁気offsetが非有限')
    return yaw_on + (math.radians(offset_deg) if magnetic == 'off' else 0.)


def initial_heading(data,snapshot,config,forced=None):
    """localへ既に設定された地球基準yawを使い、globalの補正TFを混入させない。

    READYのseedは起動時の向きなので、その後車体が旋回したbag開始時の姿勢には
    そのまま代用しない。READYを根拠にlocalのENU初期化成立を確認し、実際の
    yawは先頭local姿勢から得る。map→odomはglobal推定との方位差を含み得る。
    """
    values=snapshot.get('snapshots',{}).get('/heading_initializer_node',{}).get('values',{})
    calibration=config_values(config,'hwt905_mag_calibration.yaml','wit_mag_corrector')
    saved=values.get('offset_deg');offset=float(saved if saved is not None else calibration['offset_deg'])
    if not math.isfinite(offset):raise ValueError('磁気offsetが非有限')
    if forced is not None and not math.isfinite(forced):raise ValueError('指定初期yawが非有限')
    source='保存されたoffset_deg' if saved is not None else '保存offsetなし。現行校正値を仮定'
    if forced is not None:
        return forced,offset,dict(method='--initial-yaw-deg指定',offset_source=source,map_to_odom_applied=False)
    rows=data.get('/odometry/local',[])
    if not rows:raise ValueError('記録local姿勢なし。--initial-yaw-degでENU初期方位を指定')
    record_ns,msg=rows[0];states=[]
    for ns,status in data.get('/localization/initialization/status',[]):
        try:
            payload=json.loads(status.data)
            if isinstance(payload,dict) and 'state' in payload:states.append((ns,payload))
        except (AttributeError,ValueError,TypeError):continue
    # 記録開始直後に周期通知されるREADYも利用する。過去状態の無制限な流用はしない。
    before=[s for s in states if s[0]<=record_ns]
    chosen=max(before,key=lambda s:s[0]) if before else min(states,key=lambda s:s[0],default=None)
    if chosen is None or abs(chosen[0]-record_ns)>2_000_000_000 or chosen[1].get('state')!='READY':
        raise ValueError('bag開始時のENU初期化READYを確認できない。TFから方位を推測せず--initial-yaw-degを指定')
    try:seed=float(chosen[1]['seed_yaw_rad'])
    except (KeyError,TypeError,ValueError):raise ValueError('初期化READYに有効なseed_yaw_radがない。--initial-yaw-degを指定')
    q=msg.pose.pose.orientation;components=np.array([q.x,q.y,q.z,q.w],dtype=float)
    if not math.isfinite(seed) or not np.isfinite(components).all() or abs(np.linalg.norm(components)-1)>.01:
        raise ValueError('初期化seed/local quaternionが不正。--initial-yaw-degを指定')
    if msg.header.frame_id.lstrip('/') not in ('odom','map'):
        raise ValueError('localの座標系を確認できない: '+msg.header.frame_id+'。--initial-yaw-degを指定')
    r=quat_to_rot(q);yaw=math.atan2(r[1,0],r[0,0])
    return yaw,offset,dict(method='READY確認済みの先頭local yawを直接使用',reference_stamp=header_sec(msg),
        reference_record_ns=record_ns,offset_source=source,local_frame=msg.header.frame_id,
        initialization_state='READY',initialization_status_record_ns=chosen[0],recorded_initialization_yaw_rad=seed,
        map_to_odom_applied=False,note='静止初期化の再実行ではなくbag開始時点の疑似初期化。global補正TFの回転は加えない')
