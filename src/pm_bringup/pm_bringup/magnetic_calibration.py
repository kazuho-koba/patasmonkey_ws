"""磁気補正と方位初期化で同じ校正YAMLを使い、起動前に値を検証する。"""
import math
from pathlib import Path

import yaml


def prepare_magnetic_calibration(context, calibration_file):
    """共通YAMLのパスを解決し、磁場補正用ファイルと方位offsetを揃える。

    biasはWitのraw register単位、scaleは正の無次元倍率、方位角はdeg。
    センサを起動する前に異常値を拒否し、一部ノードだけが旧設定を使う状態を防ぐ。
    offset/declinationは磁場ベクトルには加えず、初期化ノードへ別途渡す。
    """
    path = Path(calibration_file.perform(context)).expanduser().resolve()
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
        values = document['wit_mag_corrector']['ros__parameters']
        for key in ('bias', 'scale'):
            vector = values[key]
            if not isinstance(vector, (list, tuple)) or len(vector) != 3:
                raise ValueError(key + 'は3軸分が必要です')
            if not all(math.isfinite(float(v)) for v in vector):
                raise ValueError(key + 'に非有限値があります')
        if any(float(v) <= 0 for v in values['scale']):
            raise ValueError('scaleは正の値が必要です')
        for key in ('offset_deg', 'declination_deg'):
            if not math.isfinite(float(values[key])):
                raise ValueError(key + 'に非有限値があります')
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise ValueError('磁気校正YAMLを使用できません: {}: {}'.format(path, exc)) from exc

    # node起動前に検査した共通ファイルの角度を初期化ノードへ渡す。
    # bias/scaleとは適用先を分け、磁場ベクトルへoffsetを二重適用しない。
    context.launch_configurations['_mag_offset_deg'] = str(float(values['offset_deg']))
    context.launch_configurations['_mag_declination_deg'] = str(float(values['declination_deg']))
    return path
