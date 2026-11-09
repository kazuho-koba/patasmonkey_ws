"""一括解析のbag探索・読出し・入力同定。画像本体は展開しない。"""
import hashlib
import json
from pathlib import Path
import sqlite3

import yaml


MODES = ('legacy', 'separated_offroad', 'separated_projected',
         'ekf_3d_no_accel', 'ekf_3d_accel')
TOPICS = ('/fix', '/navpvt', '/tf_static', '/tf', '/wit/imu', '/wit/mag',
          '/wheel/odometry', '/ov_msckf/odomimu', '/visual/odometry', '/vio/odometry',
          '/odometry/local', '/odometry/global', '/odometry/gps',
          '/localization/initialization/status', '/ov_msckf/diagnostics/health',
          '/vio/twist_gate/diagnostics', '/vio/vertical_gate/diagnostics')


def discover(root, excluded=()):
    """metadataだけを試行境界とし、派生出力・同一実体の重複を除く。"""
    root = Path(root).resolve()
    excluded = [Path(p).resolve() for p in excluded]
    paths = [root/'metadata.yaml'] if (root/'metadata.yaml').is_file() else root.rglob('metadata.yaml')
    return sorted({p.parent.resolve() for p in paths
                   if not any(e == p.parent.resolve() or e in p.parent.resolve().parents for e in excluded)})



def discover_incomplete(root, excluded=()):
    """storageはあるがmetadataのない候補を返す。解析単位はファイルでなくdirectory。

    未停止・コピー途中・metadata不足を区別できないため、自動修復や再生はしない。
    正常bag内部の付帯ファイルと出力directoryは、独立した試行として数えない。
    """
    root = Path(root).resolve()
    if (root / 'metadata.yaml').is_file():
        return []
    excluded = [Path(p).resolve() for p in excluded]
    candidates = {p.parent.resolve() for pattern in ('*.mcap', '*.db3')
                  for p in root.rglob(pattern) if p.is_file()}
    result = []
    for parent in candidates:
        if any(e == parent or e in parent.parents for e in excluded):
            continue
        if any((p / 'metadata.yaml').is_file()
               for p in (parent, *parent.parents) if p == root or root in p.parents):
            continue
        result.append(parent)
    return sorted(result)


def metadata(bag):
    """metadataのファイル順を尊重し、範囲外参照・不足ファイルを拒否する。"""
    bag = Path(bag).resolve()
    data = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    files = []
    for relative in data['relative_file_paths']:
        path = (bag/relative).resolve()
        if bag not in path.parents or not path.is_file():
            raise ValueError('不正または不足したbag分割ファイル: '+str(path))
        files.append(path)
    if not files:
        raise ValueError('metadataに分割ファイルがない')
    return data, files


def records(bag, topics=TOPICS):
    """MCAP/sqlite3を同じ(t記録ns, topic, type, CDR)へ変換する。"""
    from mcap.reader import make_reader
    data, paths = metadata(bag)
    for path in paths:
        if data['storage_identifier'] == 'mcap':
            with path.open('rb') as stream:
                for schema, channel, msg in make_reader(stream).iter_messages(topics=list(topics)):
                    yield msg.log_time, channel.topic, schema.name, msg.data
        elif data['storage_identifier'] == 'sqlite3':
            connection = sqlite3.connect('file:'+str(path)+'?mode=ro', uri=True)
            try:
                placeholders = ','.join('?' for _ in topics)
                query = ('SELECT m.timestamp,t.name,t.type,m.data FROM messages m '
                         'JOIN topics t ON m.topic_id=t.id WHERE t.name IN ('+placeholders+
                         ') ORDER BY m.timestamp,m.id')
                yield from connection.execute(query, tuple(topics))
            finally:
                connection.close()
        else:
            raise ValueError('未対応storage: '+data['storage_identifier'])


def messages(bag, topics=TOPICS):
    """型定義はFoxy環境のROS interfaceから解決する。"""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    types = {}
    for stamp, topic, name, raw in records(bag, topics):
        if name not in types:
            types[name] = get_message(name)
        yield stamp, topic, deserialize_message(raw, types[name])


def header_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9


def read_snapshot(bag):
    path = Path(bag)/'provenance/effective_parameters.json'
    return json.loads(path.read_text()) if path.exists() else {}


def fingerprint(bag, options, code_files):
    """大きなbagの再読込を避け、metadataとサイズ/mtime・設定・コードで再開を判定。"""
    _, paths = metadata(bag)
    payload = {'metadata': (Path(bag)/'metadata.yaml').read_text(), 'options': options,
               'files': [(str(p), p.stat().st_size, p.stat().st_mtime_ns) for p in paths],
               'code': [(str(p), hashlib.sha256(p.read_bytes()).hexdigest()) for p in code_files]}
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()
