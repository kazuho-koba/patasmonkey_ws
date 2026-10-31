"""GUIが受領済みmissionのhashと地理座標を検査することを確認する。"""
import hashlib
import json
from types import SimpleNamespace

from pm_gui.ros_backend import RosBackend


def test_mission_display_rejects_corrupted_payload():
    mission = {'waypoints': [{'latitude': 35.0, 'longitude': 139.0},
                             {'latitude': 35.1, 'longitude': 139.1}]}
    digest = hashlib.sha256(json.dumps(mission, sort_keys=True).encode()).hexdigest()
    saved = {}
    receiver = SimpleNamespace(_store=lambda name, value: saved.update({name: value}))
    payload = {'mission': mission, 'sha256': digest}
    RosBackend._mission_callback(receiver, SimpleNamespace(data=json.dumps(payload)))
    assert saved['accepted_mission'] == {'sha256': digest, 'points': [(35.0, 139.0), (35.1, 139.1)]}
    payload['mission']['waypoints'][0]['latitude'] = 36.0
    RosBackend._mission_callback(receiver, SimpleNamespace(data=json.dumps(payload)))
    assert saved['accepted_mission'] is None
