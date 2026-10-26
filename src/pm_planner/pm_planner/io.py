"""既存mission schemaと共有車両設定を読取り、入力をsnapshot化する。"""
from pathlib import Path
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


def read_limits(path):
    """実車と同じvehicle_control.yamlの制約を再利用する。"""
    p=read_yaml(path)['vehicle_interface_node']['ros__parameters']
    return Limits(float(p['min_turn_radius']),float(p['autonomous_max_linear_speed']),
                  float(p['max_yaw_rate']),float(p['max_yaw_accel']))
