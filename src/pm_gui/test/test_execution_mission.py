"""GUIが受領済みmissionのhashと地理座標を検査することを確認する。"""
import hashlib
import json
import threading
import time
from types import SimpleNamespace

from pm_gui.ros_backend import RosBackend


def test_operator_heartbeat_requires_recent_ui_activity():
    """ROS threadが生きていてもQtの更新が止まればheartbeatを送らない。"""
    sent=[]
    backend=SimpleNamespace(_lock=threading.RLock(),_operator_ui_received=None,
        _operator_session='test-session',_operator_pub=SimpleNamespace(publish=sent.append))
    RosBackend._operator_heartbeat(backend)
    assert not sent
    RosBackend.operator_activity(backend)
    RosBackend._operator_heartbeat(backend)
    assert sent[-1].data=='test-session'
    backend._operator_ui_received=time.monotonic()-2.
    RosBackend._operator_heartbeat(backend)
    assert len(sent)==1


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


def test_gui_mode_request_is_atomic_and_waits_for_ack():
    """3状態だけを要求し、未応答中に矛盾した要求を重ねない。"""
    from rclpy.task import Future
    sent=[];future=Future();responses=[]
    client=SimpleNamespace(service_is_ready=lambda:True,call_async=lambda req:sent.append(req) or future)
    backend=SimpleNamespace(_lock=threading.RLock(),_perception_mode_pending=False,
        _perception_mode_client=client,_last_response='',_service_response=responses.append)
    assert not RosBackend.request_perception_mode(backend,'terrain_without_obstacles')
    assert RosBackend.request_perception_mode(backend,'full_terrain')
    assert sent[0].mode=='full_terrain' and backend._perception_mode_pending
    assert not RosBackend.request_perception_mode(backend,'off')
    future.set_result(SimpleNamespace(success=True,mode='full_terrain',message='accepted'))
    assert responses==[future] and not backend._perception_mode_pending


def test_operator_mode_controls_show_acknowledged_state(tmp_path):
    """実Qt画面で3択と送信を確認。選択だけではUGVの実modeを変えない。"""
    import os
    import pytest
    if os.environ.get('ROS_DOMAIN_ID')!='213':pytest.skip('隔離DDS domainで実行する')
    import rclpy
    import yaml
    from pathlib import Path
    from PyQt5.QtWidgets import QApplication
    from pm_gui.operator_console import OperatorWindow
    config=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/gui.yaml').read_text())['operator_console']['ros__parameters']
    config['map'].update(online_tiles_enabled=False,tile_cache_dir=str(tmp_path/'tiles'))
    app=QApplication.instance() or QApplication([]);rclpy.init(args=[])
    backend=RosBackend(config);window=None
    try:
        window=OperatorWindow(backend,config);window._timer.stop()
        received=[];backend.request_perception_mode=lambda mode:received.append(mode)
        status=dict(state='DISABLED',drive_ready=True,reason='',readiness_reason='',perception_mode='off')
        backend._store('execution_manager',status);window._refresh()
        assert [window.perception_mode.itemData(i) for i in range(window.perception_mode.count())]==['off','obstacle_only','full_terrain']
        window.perception_mode.setCurrentIndex(2);window.perception_apply.click()
        assert received==['full_terrain'] and '認識mode: off' in window.autonomy_state.text()
        backend._store('execution_manager',dict(status,perception_mode='full_terrain'));window._refresh()
        assert '認識mode: full_terrain' in window.autonomy_state.text()
    finally:
        if window is not None:window.close();app.processEvents()
        backend.destroy_node();rclpy.shutdown()
