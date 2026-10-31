"""実車に接続せずmission受領と実行管理の通信契約を検証する。"""
import os
import copy
import time
import uuid
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool, Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path as PathMessage
from sensor_msgs.msg import NavSatFix
from pm_msgs.srv import UploadMission
from robot_localization.srv import FromLL
from pm_mission.model import load_document, to_message
from pm_mission.receiver import MissionReceiver
from pm_planner.execution_manager import ExecutionManager
from pm_planner.preview_node import PreviewNode


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


def test_upload_generates_matched_path_with_navsat(tmp_path):
    """受領→管理→FromLL→Pathを通し、新旧missionの取り違えを防ぐ。"""
    if os.environ.get('ROS_DOMAIN_ID') != '213':
        pytest.skip('ROS_DOMAIN_ID=213で隔離して実行する')
    rclpy.init(args=[])
    executor = SingleThreadedExecutor()
    receiver = MissionReceiver([Parameter('output_directory', value=str(tmp_path))])
    manager = ExecutionManager()
    preview = PreviewNode([Parameter('managed_mission_enabled', value=True),
                           Parameter('navsat_world_frame', value='map'),
                           Parameter('planning_frame', value='map')])
    helper = Node('pipeline_test')
    nodes = [receiver, manager, preview, helper]
    for node in nodes:
        executor.add_node(node)
    def convert(request, response):
        # 仮FromLLは通信と座標frameの接続確認用で、実機の地理精度は評価しない。
        response.map_point.x = (request.ll_point.longitude-139.63883757591248)*90000.
        response.map_point.y = (request.ll_point.latitude-35.8403250977535)*110000.
        return response
    helper.create_service(FromLL, '/fromLL', convert)
    upload = helper.create_client(UploadMission, '/pm/mission/upload')
    gps_pub = helper.create_publisher(Odometry, '/odometry/gps', qos_profile_sensor_data)
    fix_pub = helper.create_publisher(NavSatFix, '/fix/gated', qos_profile_sensor_data)
    paths = []
    helper.create_subscription(PathMessage, '/pm/planner/preview/reference_path', paths.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE))
    def wait(predicate):
        end = time.monotonic()+6
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        assert predicate(), 'ROS応答待ちtimeout'
    def feed_navsat():
        gps = Odometry()
        gps.header.frame_id = 'map'
        gps.header.stamp = helper.get_clock().now().to_msg()
        fix = NavSatFix()
        fix.header.stamp = gps.header.stamp
        fix.status.status = 0
        fix.latitude = 35.84
        fix.longitude = 139.63
        fix.altitude = 12.
        gps_pub.publish(gps)
        fix_pub.publish(fix)
        wait(lambda: preview.gps is not None and preview.fix is not None and
             preview.gps[0].header.stamp == gps.header.stamp and preview.fix[0].header.stamp == fix.header.stamp)
    def send(mission):
        request = UploadMission.Request()
        request.request_id = str(uuid.uuid4())
        request.mission = to_message(mission)
        future = upload.call_async(request)
        wait(future.done)
        assert future.result().accepted
    try:
        wait(lambda: upload.service_is_ready() and preview.client.service_is_ready() and
             gps_pub.get_subscription_count() > 0 and fix_pub.get_subscription_count() > 0)
        feed_navsat()
        first = load_document(Path(__file__).resolve().parents[1]/'examples/demo_mission.yaml')
        send(first)
        wait(lambda: preview.load_state == 'READY' and manager.preview is not None)
        wait(lambda: bool(paths) and len(paths[-1].poses) > 2)
        assert paths[-1].header.frame_id == 'map'
        assert manager.preview['mission_id'] == manager.mission['mission_id']
        assert manager.preview['coordinate_provenance']['mission_sha256'] == manager.mission['sha256']
        second = copy.deepcopy(first)
        second['revision'] += 1
        second['waypoints'][-1]['latitude'] += .00001
        feed_navsat()
        send(second)
        wait(lambda: manager.preview is not None and manager.preview.get('revision') == second['revision']
             and manager.preview.get('geometry_valid') and preview.load_state == 'READY')
        assert manager.preview['coordinate_provenance']['mission_sha256'] == manager.mission['sha256']
        assert manager.preview['geometry_valid'] and not manager.preview['drive_ready']
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
