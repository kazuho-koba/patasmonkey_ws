"""別DDS domainで実行する非駆動ROS接続テスト。センサや制御器は起動しない。"""
import math
import os
import time
from pathlib import Path

import pytest

rclpy=pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMessage
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import FromLL
from std_srvs.srv import Trigger
from pm_planner.preview_node import PreviewNode


ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture
def graph():
    # 実車domainへテスト用Odometryを流さない。pytestの起動側でも明示する。
    if os.environ.get('ROS_DOMAIN_ID')!='213':
        pytest.skip('ROS_DOMAIN_ID=213で隔離して実行してください')
    rclpy.init(args=[])
    executor=SingleThreadedExecutor();nodes=[]
    def add(node):nodes.append(node);executor.add_node(node);return node
    def spin_until(predicate,timeout=4.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
            if predicate():return
        assert predicate(),'ROS応答が期限内に得られませんでした'
    yield add,spin_until
    for node in nodes:executor.remove_node(node);node.destroy_node()
    executor.shutdown();rclpy.shutdown()


def parameters(**overrides):
    values=dict(mission_file=str(ROOT/'examples/demo_mission.yaml'),coordinate_mode='manual',
                anchor_file=str(ROOT/'config/home_preview_anchor.yaml'),pose_topic='/planner_test/pose')
    values.update(overrides)
    return [Parameter(k,value=v) for k,v in values.items()]


def test_manual_load_services_pose_timeout_and_reload(graph):
    add,wait=graph;planner=add(PreviewNode(parameters()))
    helper=add(Node('planner_test'))
    pub=helper.create_publisher(Odometry,'/planner_test/pose',10)
    wait(lambda:planner.load_state=='READY')
    # load後に購読するlate joinでも、有効Pathの最後のsnapshotを受信できる。
    paths=[]
    helper.create_subscription(PathMessage,'/pm/planner/preview/reference_path',paths.append,
                               QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    wait(lambda:bool(paths))
    assert paths[-1].header.frame_id=='planner_preview' and len(paths[-1].poses)>2
    wait(lambda:pub.get_subscription_count()>0)
    def pose():
        message=Odometry();message.header.frame_id='planner_preview';message.child_frame_id='base_link'
        message.header.stamp=helper.get_clock().now().to_msg()
        x,y,yaw,_=planner.reference.samples[0]
        message.pose.pose.position.x=x;message.pose.pose.position.y=y
        message.pose.pose.orientation.z=math.sin(yaw/2);message.pose.pose.orientation.w=math.cos(yaw/2)
        pub.publish(message)
    client=helper.create_client(Trigger,'/pm/planner/preview/start')
    wait(client.service_is_ready)
    missing=client.call_async(Trigger.Request());wait(missing.done)
    assert not missing.result().success
    pose();wait(lambda:planner.pose is not None)
    future=client.call_async(Trigger.Request());wait(future.done)
    assert future.result().success and planner.progress.state=='RUNNING'
    wait(lambda:planner.progress.state=='MANUAL_PAUSED',timeout=2.)
    pose();wait(lambda:not planner.pose_error)
    assert planner.progress.state=='MANUAL_PAUSED'
    assert all('Twist' not in kind for _,types in planner.get_publisher_names_and_types_by_node(
        planner.get_name(),planner.get_namespace()) for kind in types)
    assert planner.path_pub.topic_name=='/pm/planner/preview/reference_path'
    home=ROOT.parents[0]/'pm_mission/missions/home.yaml'
    planner.set_parameters([Parameter('mission_file',value=str(home))]);planner.load()
    assert planner.load_state=='READY'
    assert any(t['mode']=='symmetric_bulb' for t in planner.reference.turns)
    # homeの鋭角は今は有効なので、拒否／Path消去試験には実際に壊れたmissionを使う。
    bad=ROOT/'examples/nonexistent_mission.yaml'
    planner.set_parameters([Parameter('mission_file',value=str(bad))]);planner.load()
    assert planner.load_state=='ERROR' and planner.reference is None and planner.progress is None
    assert not planner.points
    wait(lambda:len(paths[-1].poses)==0)


def test_navsat_requires_explicit_frame_and_uses_service(graph):
    add,wait=graph
    planner=add(PreviewNode(parameters(coordinate_mode='navsat',navsat_world_frame='odom',planning_frame='odom')))
    helper=add(Node('fake_navsat'));requests=[]
    def convert(request,response):
        requests.append(request.ll_point)
        # 入力の緯度経度をメートルへ変換する実サービスに代わる、通信契約だけのfixture。
        response.map_point.x=(request.ll_point.longitude-139.63883757591248)*90000.
        response.map_point.y=(request.ll_point.latitude-35.8403250977535)*110000.
        return response
    helper.create_service(FromLL,'/fromLL',convert)
    gps=Odometry();gps.header.frame_id='wrong';gps.header.stamp=helper.get_clock().now().to_msg()
    fix=NavSatFix();fix.header.stamp=gps.header.stamp;fix.altitude=12.
    fix.latitude=35.84;fix.longitude=139.63;fix.status.status=0
    wait(lambda:planner.load_state=='WAITING_FOR_NAVSAT')
    planner.on_gps(gps);planner.on_fix(fix);planner.begin_navsat()
    assert planner.waiting and not requests and '不一致' in planner.load_error
    gps.header.frame_id='odom';planner.on_gps(gps);planner.begin_navsat()
    wait(lambda:planner.load_state in ('READY','ERROR'))
    assert planner.load_state=='READY',planner.load_error
    assert len(requests)==3 and all(r.altitude==12. for r in requests)
    assert planner.provenance['source_frame']=='odom'
    assert planner.provenance['alignment_verified'] is False


def test_invalid_yaml_is_reported_as_load_error(graph,tmp_path):
    bad=tmp_path/'broken.yaml';bad.write_text('invalid: [')
    add,wait=graph;planner=add(PreviewNode(parameters(mission_file=str(bad))))
    wait(lambda:planner.load_state=='ERROR')
    assert planner.reference is None
