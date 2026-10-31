"""実車に接続せずmission受領と実行管理の通信契約を検証する。"""
import os
import time
import uuid
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool, Trigger
from pm_msgs.srv import UploadMission
from pm_mission.model import load_document, to_message
from pm_mission.receiver import MissionReceiver
from pm_planner.execution_manager import ExecutionManager


def test_upload_start_rejected_and_stop_forwarded(tmp_path):
    if os.environ.get('ROS_DOMAIN_ID') != '213':
        pytest.skip('ROS_DOMAIN_ID=213で隔離して実行する')
    rclpy.init(args=[])
    executor = SingleThreadedExecutor()
    receiver = MissionReceiver([Parameter('output_directory', value=str(tmp_path))])
    manager = ExecutionManager()
    helper = Node('execution_test')
    nodes = [receiver, manager, helper]
    for node in nodes:
        executor.add_node(node)
    calls = []
    def disable(request, response):
        calls.append(request.data)
        response.success = True
        response.message = 'disabled'
        return response
    helper.create_service(SetBool, '/pm/vehicle/set_autonomy_enabled', disable)
    upload = helper.create_client(UploadMission, '/pm/mission/upload')
    start = helper.create_client(Trigger, '/pm/planner/execution_manager/start')
    stop = helper.create_client(Trigger, '/pm/planner/execution_manager/stop')
    def wait(predicate):
        end = time.monotonic()+5
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        assert predicate(), 'ROS応答待ちtimeout'
    try:
        wait(lambda: upload.service_is_ready() and start.service_is_ready() and stop.service_is_ready()
             and manager.vehicle_client.service_is_ready())
        future = start.call_async(Trigger.Request())
        wait(future.done)
        assert not future.result().success and '未受領' in future.result().message
        source = Path(__file__).resolve().parents[1]/'examples/demo_mission.yaml'
        mission = load_document(source)
        request = UploadMission.Request()
        request.request_id = str(uuid.uuid4())
        request.mission = to_message(mission)
        future = upload.call_async(request)
        wait(future.done)
        assert future.result().accepted and Path(future.result().stored_path).is_file()
        wait(lambda: manager.mission is not None)
        assert manager.mission_document == mission
        assert manager.mission['mission_id'] == mission['mission_id']
        future = start.call_async(Trigger.Request())
        wait(future.done)
        assert not future.result().success and 'MPPI' in future.result().message
        assert calls == []
        future = stop.call_async(Trigger.Request())
        wait(future.done)
        assert future.result().success
        wait(lambda: calls == [False] and manager.stop_future.done())
        # 受信箱の再起動後も、保存済みfileとhashを検証して同じ経路を復元する。
        executor.remove_node(receiver)
        receiver.destroy_node()
        manager.mission = None
        manager.mission_document = None
        receiver = MissionReceiver([Parameter('output_directory', value=str(tmp_path))])
        nodes[0] = receiver
        executor.add_node(receiver)
        wait(lambda: manager.mission_document == mission)
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
