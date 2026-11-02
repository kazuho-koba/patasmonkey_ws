"""既存mission schemaと共有車両設定を読取り、入力をsnapshot化する。"""
from pathlib import Path
import math
import yaml
from pm_mission.model import load_document, validate
from .geodesy import Anchor
from .reference import Limits


def read_yaml(path):
    """構文エラーも通常の入力エラーとして呼出側へ返す。"""
    try:return yaml.safe_load(Path(path).expanduser().read_text())
    except yaml.YAMLError as exc:raise ValueError('YAML構文エラー: '+str(exc)) from exc


def read_mission(path):
    """編集途中のfileは走行用previewに採用せず、送信可能条件まで検証する。"""
    try:return validate(load_document(Path(path).expanduser()),for_submission=True)
    except yaml.YAMLError as exc:raise ValueError('mission YAML構文エラー: '+str(exc)) from exc


def read_anchor(path):
    """暗黙の基準点を作らず、明示fileのanchorだけを使う。"""
    document=read_yaml(path)
    if not isinstance(document,dict):raise ValueError('anchor YAMLはmappingが必要です')
    return Anchor(**document)


def read_vehicle_parameters(path):
    """共通の車両設定と隣接するgeometryを、launchと同じ順番で読む。

    geometryがない単独設定ではvehicle interfaceの宣言既定を使用する。
    motor接続やROS nodeの初期化は行わない。
    """
    p=dict(wheel_radius=.1,tread_width=.36,gear_ratio=10.,max_whl_rps=4.,
           skid_steer_yaw_efficiency=1.,max_linear_speed=1.)
    geometry=Path(path).expanduser().parent/'vehicle_geometry.yaml'
    if geometry.is_file():p.update(read_yaml(geometry)['/**']['ros__parameters'])
    p.update(read_yaml(path)['vehicle_interface_node']['ros__parameters'])
    keys=('wheel_radius','tread_width','gear_ratio','max_whl_rps','skid_steer_yaw_efficiency',
          'max_linear_speed','min_turn_radius','autonomous_max_linear_speed','max_yaw_rate','max_yaw_accel')
    for key in keys:
        p[key]=float(p[key])
        if not math.isfinite(p[key]) or p[key]<=0:
            raise ValueError(key+'はplanner用に正の有限値が必要です（制限無効化は未対応）')
    if p['skid_steer_yaw_efficiency']>1:raise ValueError('旋回効率は1以下が必要です')
    return p


def read_limits(path):
    """YAMLの車体側制約をそのまま使う。スキッド補正はinterfaceが担当する。

    内輪の逆転可否もinterface側の仕様に従い、輪距や旋回効率から半径を
    暗黙に拡大しない。車輪の物理上限による速度の絶対上限だけは保持する。
    """
    p=read_vehicle_parameters(path)
    speed=min(p['autonomous_max_linear_speed'],p['max_linear_speed'],
              2.*math.pi*p['wheel_radius']*p['max_whl_rps'])
    return Limits(p['min_turn_radius'],speed,p['max_yaw_rate'],p['max_yaw_accel'])
