"""WGS84経路documentの検証・永続化。Qt/ROSには依存しない。"""
import copy
import math
import os
from pathlib import Path
import tempfile
import uuid

import yaml

ATTRIBUTES = ('pass', 'start', 'goal', 'pause', 'confirm')
LABELS = ('通過', 'スタート', 'ゴール', '一時停止', 'オペレータ確認')
MAX_POINTS = 10000
MAX_FILE_BYTES = 16 * 1024 * 1024


def new_mission():
    """編集途中の空ミッションを生成する。識別子は保存・読み込みでも維持する。"""
    return {'schema_version': 1, 'coordinate_system': 'WGS84',
            'mission_id': str(uuid.uuid4()), 'name': '新しいミッション',
            'revision': 0, 'waypoints': []}


def validate(document, for_submission=False):
    """境界で内容を検証し、正規化した独立documentを返す。

    保存時は編集途中も許可する。送信時は先頭START/末尾GOALを要求し、
    中間点のSTART/GOALを拒否する。WGS84をmap/odom座標へ変換しない。
    """
    if (not isinstance(document, dict)
            or type(document.get('schema_version')) is not int
            or document.get('schema_version') != 1):
        raise ValueError('対応しないmission schemaです')
    if document.get('coordinate_system') != 'WGS84':
        raise ValueError('座標系はWGS84に限定します')
    uuid.UUID(str(document.get('mission_id', '')))
    name = document.get('name')
    if not isinstance(name, str) or not name.strip() or len(name) > 200:
        raise ValueError('ミッション名は1～200文字です')
    revision = document.get('revision')
    if type(revision) is not int or not 0 <= revision < 2**64:
        raise ValueError('revisionが不正です')
    points = document.get('waypoints')
    if not isinstance(points, list) or len(points) > MAX_POINTS:
        raise ValueError('経路点は最大{}点です'.format(MAX_POINTS))
    result = copy.deepcopy(document)
    ids = set()
    for index, point in enumerate(result['waypoints']):
        if not isinstance(point, dict):
            raise ValueError('経路点{}が不正です'.format(index + 1))
        point_id = str(point.get('waypoint_id', ''))
        uuid.UUID(point_id)
        if point_id in ids:
            raise ValueError('経路点IDが重複しています')
        ids.add(point_id)
        for key, limit in (('latitude', 90), ('longitude', 180)):
            value = float(point[key])
            if not math.isfinite(value) or abs(value) > limit:
                raise ValueError('経路点{}の{}が範囲外です'.format(index + 1, key))
            point[key] = value
        if point.get('attribute') not in ATTRIBUTES:
            raise ValueError('経路点の属性が不正です')
        pause = float(point.get('pause_sec', 0))
        if not math.isfinite(pause) or not 0 <= pause <= 86400:
            raise ValueError('一時停止時間は0～86400秒です')
        point['pause_sec'] = pause
        note = point.get('note', '')
        if not isinstance(note, str) or len(note) > 2000:
            raise ValueError('備考は2000文字までです')
        point['note'] = note
    if for_submission:
        if len(points) < 2:
            raise ValueError('送信には2点以上必要です')
        kinds = [point['attribute'] for point in points]
        if kinds[0] != 'start' or kinds[-1] != 'goal':
            raise ValueError('先頭をスタート、末尾をゴールに設定してください')
        if any(kind in ('start', 'goal') for kind in kinds[1:-1]):
            raise ValueError('スタート・ゴールは先頭・末尾の1点ずつにしてください')
    return result


def save_document(path, document):
    """同一directory内の一時fileをfsyncして置換し、途中書き込みを防ぐ。"""
    path = Path(path).expanduser()
    payload = yaml.safe_dump(validate(document), allow_unicode=True, sort_keys=False)
    if len(payload.encode('utf-8')) > MAX_FILE_BYTES:
        raise ValueError('ミッションfileは16 MiB以下にしてください')
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile('w', dir=str(path.parent), delete=False,
                                         encoding='utf-8') as stream:
            temporary = stream.name
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, str(path))
        temporary = None
        directory_fd = os.open(str(path.parent), os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if temporary is not None:
            os.unlink(temporary)


def load_document(path):
    """誤った座標系や巨大fileを読み込まず、YAMLを検証して返す。"""
    path = Path(path).expanduser()
    if path.stat().st_size > MAX_FILE_BYTES:
        raise ValueError('ミッションfileは16 MiB以下にしてください')
    return validate(yaml.safe_load(path.read_text(encoding='utf-8')))


def to_message(document):
    """検証済みdocumentを型付きROS interfaceへ変換する。"""
    from pm_msgs.msg import Mission, MissionWaypoint
    document = validate(document, for_submission=True)
    message = Mission()
    for key in ('schema_version', 'mission_id', 'name', 'revision'):
        setattr(message, key, document[key])
    for point in document['waypoints']:
        waypoint = MissionWaypoint()
        for key in ('waypoint_id', 'latitude', 'longitude', 'pause_sec', 'note'):
            setattr(waypoint, key, point[key])
        waypoint.attribute = ATTRIBUTES.index(point['attribute'])
        message.waypoints.append(waypoint)
    return message


def from_message(message):
    """受領したROS messageも信頼せず、同じdocument検証を適用する。"""
    if len(message.waypoints) > MAX_POINTS:
        raise ValueError('経路点数の上限を超えています')
    points = []
    for point in message.waypoints:
        if point.attribute >= len(ATTRIBUTES):
            raise ValueError('未知の属性番号です')
        points.append({'waypoint_id': point.waypoint_id, 'latitude': point.latitude,
                       'longitude': point.longitude, 'attribute': ATTRIBUTES[point.attribute],
                       'pause_sec': point.pause_sec, 'note': point.note})
    return validate({'schema_version': message.schema_version, 'coordinate_system': 'WGS84',
                     'mission_id': message.mission_id, 'name': message.name,
                     'revision': message.revision, 'waypoints': points}, True)
