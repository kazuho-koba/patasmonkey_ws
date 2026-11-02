"""隔離domainで遠隔受付とQtの表示・終了を実プロセスで確認する。"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

from test_inspection import fixture


@pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID') != '229', reason='隔離domain 229専用')
def test_remote_agent_and_qt(tmp_path):
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from pm_msgs.srv import InspectBags
    from std_msgs.msg import String
    from PyQt5.QtWidgets import QApplication
    from pm_inspector.app import Window

    bag = fixture(tmp_path)
    binary = Path(get_package_prefix('pm_inspector'))/'lib/pm_inspector/inspection_agent'
    log = (tmp_path/'agent.log').open('w')
    child = subprocess.Popen([str(binary), '--ros-args', '-p', 'bag_root:='+str(tmp_path),
                              '-p', 'result_root:='+str(tmp_path/'reports')], stdout=log, stderr=log)
    rclpy.init()
    node = rclpy.create_node('inspection_integration')
    client = node.create_client(InspectBags, '/pm/inspector/request')
    publisher = node.create_publisher(String, '/pm/robot_manager/mission_bag/status', 10)

    def call(operation, data):
        request = InspectBags.Request()
        request.operation, request.payload = operation, json.dumps(data)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.)
        assert future.done()
        result = future.result()
        return result.success, json.loads(result.payload)

    window = None
    try:
        assert client.wait_for_service(timeout_sec=15.)
        success, result = call('list', {})
        assert success and str(bag) in result['bags']
        success, result = call('list', dict(root=str(tmp_path.parent)))
        assert not success
        deadline = time.monotonic()+10
        while publisher.get_subscription_count() == 0:
            assert time.monotonic() < deadline
            rclpy.spin_once(node, timeout_sec=.1)
        publisher.publish(String(data=json.dumps(dict(profile='mission', state='RECORDING', output=str(bag)))))
        time.sleep(.3)
        assert not call('start', dict(bags=[str(bag)]))[0]
        publisher.publish(String(data=json.dumps(dict(profile='mission', state='STOPPED', output=str(bag)))))
        time.sleep(.3)
        assert call('start', dict(bags=[str(bag)]))[0]
        deadline = time.monotonic()+15
        while True:
            success, status = call('status', {})
            assert success
            if status['state'] != 'RUNNING':
                break
            assert time.monotonic() < deadline
            time.sleep(.1)
        assert status['state'] == 'DONE', status
        success, report = call('result', dict(index=0))
        assert success and report['integrity_verified']
        assert 'window_hz' not in report['topics']['/imu']
        app = QApplication.instance() or QApplication([])
        window = Window(dict(service='/pm/inspector/request', remote_root=str(tmp_path), request_timeout_sec=2))
        window.show()
        window.list_bags()
        deadline = time.monotonic()+5
        while window.busy:
            app.processEvents()
            assert time.monotonic() < deadline
            time.sleep(.01)
        assert window.list.count() >= 1
        window.send('status', {})
        deadline = time.monotonic()+5
        while window.busy:
            app.processEvents()
            assert time.monotonic() < deadline
            time.sleep(.01)
        # statusの完了通知がresult要求を連鎖させるため、表が埋まるまで待つ。
        while window.table.rowCount() == 0:
            app.processEvents()
            assert time.monotonic() < deadline
            time.sleep(.01)
        assert window.table.item(0, 0).text() == '/imu'
        window.topic_result(0, 0)
        while window.busy:
            app.processEvents()
            assert time.monotonic() < deadline
            time.sleep(.01)
        assert window.timeline.values
        window.grab().save(str(tmp_path/'inspector.png'))
    finally:
        if window is not None:
            window.close()
        node.destroy_node()
        rclpy.shutdown()
        child.send_signal(signal.SIGINT)
        child.wait(timeout=15.)
        log.close()


@pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID') != '229', reason='隔離domain 229専用')
def test_console_button_is_next_to_mission_planner(tmp_path, monkeypatch):
    import rclpy
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import QProcess
    from pm_gui.operator_console import OperatorWindow
    from pm_gui.ros_backend import RosBackend
    path = Path(get_package_share_directory('pm_gui'))/'config/gui.yaml'
    config = yaml.safe_load(path.read_text())['operator_console']['ros__parameters']
    config['map'].update(online_tiles_enabled=False, tile_cache_dir=str(tmp_path/'tiles'))
    app = QApplication.instance() or QApplication([])
    rclpy.init()
    backend = RosBackend(config)
    window = None
    calls = []
    monkeypatch.setattr(QProcess, 'startDetached', lambda *args: calls.append(args) or True)
    try:
        window = OperatorWindow(backend, config)
        window._timer.stop()
        row = window.centralWidget().layout().itemAt(0).layout()
        assert row.itemAt(1).widget().text() == 'MISSION PLANNER'
        button = row.itemAt(2).widget()
        assert button.text() == 'BAG INSPECTOR'
        button.click()
        assert calls == [('ros2', ['launch', 'pm_inspector', 'bag_inspector.launch.py'])]
    finally:
        if window:
            window.close()
            app.processEvents()
        backend.destroy_node()
        rclpy.shutdown()
