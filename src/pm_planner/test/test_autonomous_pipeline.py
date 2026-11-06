"""隔離ROS graphで受領→地理変換→MPPI→自律速度と停止・再開を確認する。

車両はメモリ内のserviceと数値モデルのみ。ROS/ODrive driverを起動しない。
"""
import copy
import importlib.util
import json
import math
import os
from pathlib import Path
import time
import uuid
import pytest

rclpy=pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry,Path as PathMessage
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger,SetBool
from pm_msgs.srv import UploadMission,SetPlannerMode
from robot_localization.srv import FromLL
from pm_mission.receiver import MissionReceiver
from pm_mission.model import load_document,to_message
from pm_planner.preview_node import PreviewNode
from pm_planner.execution_manager import ExecutionManager
from pm_planner.io import read_anchor,read_vehicle_parameters


class Rig:
    def __init__(self,tmp_path,manager_parameters=()):
        self.ws=Path(__file__).resolve().parents[3]
        self.executor=SingleThreadedExecutor();self.permission=False;self.calls=[];self.command=(0.,0.)
        self.time=100.;self.x=self.y=self.yaw=self.v=self.w=0.;self.commands=[];self.trajectories=[];self.paths=[]
        sim=Parameter('use_sim_time',value=True)
        self.receiver=MissionReceiver([Parameter('output_directory',value=str(tmp_path))])
        self.manager=ExecutionManager([sim,Parameter('operator_timeout_sec',value=.3),
            Parameter('tracking_mode',value='corridor'),Parameter('max_deviation_m',value=2.)]+list(manager_parameters))
        self.preview=PreviewNode([sim,Parameter('managed_mission_enabled',value=True),
             Parameter('navsat_world_frame',value='map'),Parameter('planning_frame',value='map'),
             Parameter('pose_topic',value='/odometry/global')])
        self.helper=Node('pipeline_vehicle_sim',parameter_overrides=[sim])
        self.nodes=[self.receiver,self.manager,self.preview,self.helper]
        for n in self.nodes:self.executor.add_node(n)
        self.manager.control_timer.cancel();self.preview.timer.cancel()
        self.clock=self.helper.create_publisher(Clock,'/clock',10)
        self.odom=self.helper.create_publisher(Odometry,'/odometry/global',qos_profile_sensor_data)
        self.gps=self.helper.create_publisher(Odometry,'/odometry/gps',qos_profile_sensor_data)
        self.fix=self.helper.create_publisher(NavSatFix,'/fix/gated',qos_profile_sensor_data)
        self.operator=self.helper.create_publisher(String,'/pm/planner/operator_heartbeat',1)
        self.vehicle=self.helper.create_publisher(String,'/pm/vehicle/status',10)
        self.joy=self.helper.create_publisher(Twist,'/cmd_vel_joy',10)
        self.permission_service=self.helper.create_service(SetBool,'/pm/vehicle/set_autonomy_enabled',self.allow)
        anchor=read_anchor(self.ws/'src/pm_planner/config/home_preview_anchor.yaml')
        def convert(req,res):
            res.map_point.x,res.map_point.y=anchor.project(req.ll_point.latitude,req.ll_point.longitude)
            return res
        self.helper.create_service(FromLL,'/fromLL',convert)
        self.helper.create_subscription(Twist,'/cmd_vel_autonomy',self.receive_command,1)
        self.helper.create_subscription(String,'/pm/planner/execution_manager/trajectory',
            lambda m:self.trajectories.append(json.loads(m.data)),10)
        self.helper.create_subscription(PathMessage,'/pm/planner/execution_manager/local_path',self.paths.append,10)
        self.upload=self.helper.create_client(UploadMission,'/pm/mission/upload')
        self.start=self.helper.create_client(Trigger,'/pm/planner/execution_manager/start')
        self.stop=self.helper.create_client(Trigger,'/pm/planner/execution_manager/stop')
        spec=importlib.util.spec_from_file_location('plant',self.ws/'src/pm_evaluation/tools/mppi_robustness.py')
        self.plant=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.plant)
        self.control=self.plant.make_interface(self.ws/'src/pm_control/pm_control/vehicle_interface_node.py',
                          read_vehicle_parameters(self.ws/'src/pm_config/config/vehicle_control.yaml'))

    def allow(self,req,res):
        self.permission=req.data;self.calls.append(req.data);res.success=True;res.message='simulated vehicle permission';return res

    def receive_command(self,msg):
        self.command=(msg.linear.x,msg.angular.z);self.commands.append(self.command)

    def pump(self,n=25):
        for _ in range(n):self.executor.spin_once(timeout_sec=.0002)

    def feed(self,heartbeat=True,pose=True,move=False):
        self.time+=.1
        clock=Clock();clock.clock.sec=int(self.time);clock.clock.nanosec=int(round((self.time-int(self.time))*1e9))
        if clock.clock.nanosec==1000000000:clock.clock.sec+=1;clock.clock.nanosec=0
        self.clock.publish(clock);self.pump()
        # 人工車体の初期設置方向を経路へ合わせる。実車の姿勢変更を模擬する処理ではない。
        if self.manager.follower.snapshot and self.manager.pose_history is None:
            self.yaw=self.manager.follower.reference.samples[0][2]
        msg=Odometry();msg.header.stamp=clock.clock;msg.header.frame_id='map';msg.child_frame_id='base_link'
        msg.pose.pose.position.x=self.x;msg.pose.pose.position.y=self.y
        msg.pose.pose.orientation.z=math.sin(self.yaw/2);msg.pose.pose.orientation.w=math.cos(self.yaw/2)
        msg.twist.twist.linear.x=self.v;msg.twist.twist.angular.z=self.w
        if pose:self.odom.publish(msg)
        self.gps.publish(msg)
        fix=NavSatFix();fix.header.stamp=clock.clock;fix.status.status=0
        fix.latitude=35.84;fix.longitude=139.63;fix.altitude=0.;self.fix.publish(fix)
        if heartbeat:self.operator.publish(String(data='test_operator_session'))
        self.vehicle.publish(String(data=json.dumps(dict(connected=True,armed=True,emergency_stop=False,autonomy_enabled=self.permission))))
        self.pump();self.preview.tick();self.pump();self.manager.tick();self.pump()
        if move:
            v,w=self.plant.apply(self.control,self.command if self.permission else (0.,0.),.1)
            self.v,self.w=float(v),float(w);self.yaw+=w*.1
            self.x+=v*math.cos(self.yaw)*.1;self.y+=v*math.sin(self.yaw)*.1

    def until(self,predicate,timeout=8,**kwargs):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.feed(**kwargs)
            if predicate():return
        assert predicate(),'pipeline timeout: '+self.manager.reason+' / '+self.manager.readiness()

    def send(self,mission):
        req=UploadMission.Request();req.request_id=uuid.uuid4().hex;req.mission=to_message(mission)
        f=self.upload.call_async(req);self.until(f.done)
        assert f.result().accepted
        self.until(lambda:self.manager.follower.reference is not None and self.preview.load_state=='READY')
        self.yaw=self.manager.follower.reference.samples[0][2]
        self.until(lambda:not self.manager.readiness())

    def begin(self):
        f=self.start.call_async(Trigger.Request());self.until(f.done)
        assert f.result().success,f.result().message
        self.until(lambda:self.manager.state=='RUNNING')

    def close(self):
        self.manager.halt('test shutdown');self.pump()
        for n in self.nodes:self.executor.remove_node(n);n.destroy_node()
        self.executor.shutdown()


@pytest.fixture
def rig(tmp_path):
    if os.environ.get('ROS_DOMAIN_ID')!='213':pytest.skip('ROS_DOMAIN_ID=213で隔離して実行する')
    rclpy.init(args=[]);r=Rig(tmp_path)
    try:yield r
    finally:r.close();rclpy.shutdown()


def test_home_upload_to_global_path_local_trajectory_and_completion(rig):
    r=rig
    mission=load_document(r.ws/'src/pm_mission/missions/home.yaml')
    r.until(lambda:r.upload.service_is_ready() and r.start.service_is_ready() and r.manager.vehicle_client.service_is_ready())
    r.send(mission)
    assert all(v==0. and w==0. for v,w in r.commands)
    assert r.manager.follower.snapshot['mission_id']==mission['mission_id']
    assert r.manager.follower.snapshot['frame_id']=='map'
    r.begin()
    start_time=r.time
    for _ in range(5000):
        r.feed(move=True)
        if r.manager.state!='RUNNING':break
    assert r.manager.state=='COMPLETED',r.manager.reason
    r.until(lambda:not r.permission)
    assert r.command==(0.,0.)
    assert len(r.manager.follower.progress.completed)==len(mission['waypoints'])
    assert any(v>0. for v,w in r.commands)
    assert any(p.poses and p.header.frame_id=='map' for p in r.paths)
    trajectory=next(x for x in r.trajectories if x.get('valid') and len(x['points'])>2)
    assert trajectory['path_id']==r.manager.follower.snapshot['path_id']
    assert trajectory['points'][0]['time_from_start_sec']==pytest.approx(.1)
    assert all(0.<=q['v']<=1.+1e-8 and abs(q['w'])<=1.+1e-8 for q in trajectory['points'])
    if os.environ.get('PM_PLANNER_TEST_REPORT_DIR'):
        output=Path(os.environ['PM_PLANNER_TEST_REPORT_DIR']);output.mkdir(parents=True,exist_ok=True)
        (output/'home_pipeline.json').write_text(json.dumps(dict(
            mission_id=mission['mission_id'],waypoints_completed=len(r.manager.follower.progress.completed),
            waypoints_total=len(mission['waypoints']),path_length_m=r.manager.follower.reference.length,
            duration_sim_sec=r.time-start_time,state=r.manager.state,vehicle_permission=r.permission,
            final_command=r.command,trajectory_frame=trajectory['frame_id'],path_id=trajectory['path_id']),indent=2))


def test_manual_pose_and_gui_loss_latch_until_explicit_start(rig):
    r=rig
    mission=load_document(r.ws/'src/pm_planner/examples/demo_mission.yaml')
    r.until(lambda:r.upload.service_is_ready() and r.start.service_is_ready() and r.manager.vehicle_client.service_is_ready())
    r.send(mission);r.begin()
    for _ in range(12):r.feed(move=True)
    r.joy.publish(Twist());r.pump();r.feed(move=True)
    assert r.manager.state=='DISABLED' and r.command==(0.,0.)

    for _ in range(5):r.feed(move=True)
    assert r.manager.state=='DISABLED'
    r.begin();r.feed(move=True)
    for _ in range(7):r.feed(pose=False,move=True)
    assert r.manager.state=='DISABLED' and r.command==(0.,0.)
    for _ in range(4):r.feed(move=True)
    assert r.manager.state=='DISABLED'
    r.begin();r.feed(move=True)
    # GUI期限はwall clock、人工車体は加速したROS clockで動く。
    # 切断の確認中は車体を静止させ、経路走破や旋回FAULTとの競合を避ける。
    r.v=r.w=0.
    end=time.monotonic()+.4
    while time.monotonic()<end:r.feed(heartbeat=False,move=False)
    # 期限切れが計算前ならDISABLED、計算終了後ならFAULT。
    # どちらでもゼロ・許可解除・明示再開が必要という契約を確認する。
    assert r.manager.state in ('DISABLED','FAULT') and r.command==(0.,0.),r.manager.reason
    assert 'heartbeat' in r.manager.reason and not r.manager.desired_permission
    for _ in range(3):r.feed(move=True)
    assert r.manager.state in ('DISABLED','FAULT')
    r.begin()
    stop=r.stop.call_async(Trigger.Request());r.until(stop.done,move=True)
    assert stop.result().success
    r.until(lambda:not r.permission,move=True)
    assert r.manager.state=='DISABLED' and r.command==(0.,0.)

def test_late_enable_response_cannot_resume_after_stop(rig):
    """開始serviceの応答待ちで停止した場合も、古いtrue応答で有効化しない。"""
    from rclpy.task import Future
    r=rig;requests=[];futures=[]
    class Client:
        def service_is_ready(self):return True
        def call_async(self,request):
            requests.append(request.data);future=Future();futures.append(future);return future
    r.manager.vehicle_client=Client()
    r.manager.state='STARTING';r.manager.desired_permission=True
    r.manager.request_permission(force=True)
    r.manager.stop(Trigger.Request(),Trigger.Response())
    assert requests==[True]  # falseは先のtrue要求が終わるまで直列待機する。
    futures[0].set_result(SetBool.Response(success=True,message='late enabled'))
    assert requests==[True,False]
    assert r.manager.state=='DISABLED' and not r.manager.permission_ack
    futures[1].set_result(SetBool.Response(success=True,message='disabled'))
    r.manager.tick();r.pump()
    assert r.command==(0.,0.)


def test_new_mission_and_old_reference_disable_velocity(rig):
    """走行中のmission差替えと遅れて到着した旧Pathで旧速度を再使用しない。"""
    r=rig
    mission=load_document(r.ws/'src/pm_planner/examples/demo_mission.yaml')
    r.until(lambda:r.upload.service_is_ready() and r.start.service_is_ready() and r.manager.vehicle_client.service_is_ready())
    r.send(mission);r.begin()
    for _ in range(6):r.feed(move=True)
    old=copy.deepcopy(r.manager.follower.snapshot)
    changed=copy.deepcopy(mission);changed['revision']+=1;changed['waypoints'][-1]['latitude']+=.00001
    req=UploadMission.Request();req.request_id=uuid.uuid4().hex;req.mission=to_message(changed)
    future=r.upload.call_async(req);r.until(future.done,move=True)
    r.until(lambda:r.manager.mission['revision']==changed['revision'] and r.manager.follower.snapshot is not None,move=True)
    assert r.manager.state=='DISABLED' and r.command==(0.,0.)
    assert r.manager.follower.snapshot['mission_sha256']!=old['mission_sha256']
    r.manager.on_reference(String(data=json.dumps(old)));r.pump()
    assert r.manager.follower.snapshot is None
    assert r.manager.state=='DISABLED' and r.command==(0.,0.)


def test_resume_rejects_covariance_rotation_and_pose_jump(rig):
    """手動停止中にも推定品質を検査し、不自然なジャンプは保持する。"""
    r=rig;mission=load_document(r.ws/'src/pm_planner/examples/demo_mission.yaml')
    r.until(lambda:r.upload.service_is_ready() and r.start.service_is_ready() and r.manager.vehicle_client.service_is_ready())
    r.send(mission);r.begin();r.manager.stop(Trigger.Request(),Trigger.Response());r.feed()
    msg=Odometry();msg.header.frame_id='map';msg.child_frame_id='base_link'
    msg.header.stamp=r.manager.get_clock().now().to_msg()
    msg.pose.pose.position.x=r.x;msg.pose.pose.position.y=r.y
    msg.pose.pose.orientation.z=math.sin(r.yaw/2);msg.pose.pose.orientation.w=math.cos(r.yaw/2)
    msg.pose.covariance[0]=4.
    r.manager.on_pose(msg)
    result=r.manager.start(Trigger.Request(),Trigger.Response())
    assert not result.success and '不確かさ' in result.message
    r.feed();msg.pose.covariance[0]=0.;msg.header.stamp=r.manager.get_clock().now().to_msg()
    msg.twist.twist.angular.z=.2;r.manager.on_pose(msg)
    result=r.manager.start(Trigger.Request(),Trigger.Response())
    assert not result.success and '旋回停止' in result.message
    r.feed();msg.twist.twist.angular.z=0.;msg.header.stamp=r.manager.get_clock().now().to_msg()
    msg.pose.pose.position.x+=10.;r.manager.on_pose(msg)
    result=r.manager.start(Trigger.Request(),Trigger.Response())
    assert not result.success and r.manager.pose_fault
    r.feed();assert r.manager.pose_fault and r.manager.state=='DISABLED'


def test_resume_rejoins_after_manual_advance(rig):
    """GUI停止後に手動で先行しても、明示開始時にsolver弧長を同期する。"""
    r=rig;mission=load_document(r.ws/'src/pm_planner/examples/demo_mission.yaml')
    r.until(lambda:r.upload.service_is_ready() and r.start.service_is_ready() and r.manager.vehicle_client.service_is_ready())
    r.send(mission);r.begin();r.manager.stop(Trigger.Request(),Trigger.Response());r.feed()
    samples=r.manager.follower.reference.samples
    # 経路の約3 m先まで連続的に手動移動。位置ジャンプを作らず、中断中の監視も通す。
    for x,y,yaw,s in samples:
        if s>3.:break
        r.x=x;r.y=y;r.yaw=yaw;r.v=0.;r.w=0.;r.feed()
    before=r.manager.follower.progress.station
    r.begin()
    assert r.manager.follower.progress.station>before+2.
    assert r.manager.follower.solver.station>=r.manager.follower.progress.station-.2


def test_terrain_mode_requires_fresh_clear_map_and_stops_on_unknown(rig):
    """任意のperception入力が開始条件・MPPI・停止へ接続されていることを確認。"""
    from nav_msgs.msg import OccupancyGrid
    r=rig;mission=load_document(r.ws/'src/pm_planner/examples/demo_mission.yaml')
    r.until(lambda:r.upload.service_is_ready() and r.start.service_is_ready() and r.manager.vehicle_client.service_is_ready())
    r.send(mission);r.manager.set_perception_mode(SetPlannerMode.Request(mode='full_terrain'),SetPlannerMode.Response())
    assert 'costmap' in r.manager.readiness()
    msg=OccupancyGrid();msg.header.frame_id='map';msg.header.stamp=r.manager.get_clock().now().to_msg()
    msg.info.width=120;msg.info.height=120;msg.info.resolution=.05
    msg.info.origin.position.x=-3.;msg.info.origin.position.y=-3.;msg.info.origin.orientation.w=1.
    msg.data=[0]*(120*120);r.manager.install_terrain(msg)
    assert not r.manager.readiness()
    r.begin()
    msg.data=[-1]*(120*120);msg.header.stamp=r.manager.get_clock().now().to_msg();r.manager.install_terrain(msg)
    r.feed()
    assert r.manager.state=='WAITING' and r.command==(0.,0.)
    assert r.permission and 'footprint' in r.manager.reason
    for _ in range(12):r.feed()
    assert '期限切れ' in r.manager.readiness()


def test_terrain_transform_uses_grid_stamp_and_rejects_missing_tf(rig):
    """odomのgridをmapへ揃える際にlatest TFへ代用しない。"""
    from types import SimpleNamespace
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import Transform
    r=rig;mission=load_document(r.ws/'src/pm_planner/examples/demo_mission.yaml')
    r.until(lambda:r.upload.service_is_ready() and r.start.service_is_ready() and r.manager.vehicle_client.service_is_ready())
    r.send(mission)
    msg=OccupancyGrid();msg.header.frame_id='odom';msg.header.stamp=r.manager.get_clock().now().to_msg()
    msg.info.width=120;msg.info.height=120;msg.info.resolution=.05
    msg.info.origin.position.x=-3.;msg.info.origin.position.y=-3.;msg.info.origin.orientation.w=1.
    msg.data=[0]*14400;calls=[]
    tf=Transform();tf.rotation.w=1.;tf.translation.x=1.
    def lookup(target,source,stamp):
        calls.append((target,source,stamp.nanoseconds));return SimpleNamespace(transform=tf)
    r.manager.tf_buffer=SimpleNamespace(lookup_transform=lookup)
    r.manager.install_terrain(msg)
    assert r.manager.terrain is not None
    assert r.manager.terrain.origin[:2]==pytest.approx((-2.,-3.))
    assert calls==[('map','odom',msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec)]
    def unavailable(*args):raise RuntimeError('TF unavailable')
    r.manager.tf_buffer=SimpleNamespace(lookup_transform=unavailable)
    r.manager.install_terrain(msg)
    assert r.manager.terrain is None and 'TF unavailable' in r.manager.terrain_error


def test_received_visit_is_physically_approached_before_completion(rig):
    """VISIT属性を実際の受領serviceから渡し、必須点の半径内通過を確認する。"""
    r=rig;mission=load_document(r.ws/'src/pm_planner/examples/demo_mission.yaml')
    mission['waypoints'][1]['attribute']='visit'
    r.until(lambda:r.upload.service_is_ready() and r.start.service_is_ready() and r.manager.vehicle_client.service_is_ready())
    r.send(mission)
    visit=r.manager.follower.reference.events[1];closest=float('inf');r.begin()
    for _ in range(1200):
        r.feed(move=True);closest=min(closest,math.hypot(r.x-visit['x'],r.y-visit['y']))
        if r.manager.state!='RUNNING':break
    assert r.manager.state=='COMPLETED',r.manager.reason
    assert closest<=r.manager.p['pass_radius_m']
    assert visit['waypoint_id'] in r.manager.follower.progress.completed
    assert visit['waypoint_id'] not in r.manager.follower.progress.inferred_completed


def test_mode_service_selects_independent_cost_and_latches_changes(rig):
    """DDSの新interfaceからmode変更と独立mapを入力し、停止・再開まで確認する。"""
    from pm_msgs.srv import SetPlannerMode
    from pm_msgs.msg import TerrainAssessment
    r=rig;mission=load_document(r.ws/'src/pm_planner/examples/demo_mission.yaml')
    r.until(lambda:r.upload.service_is_ready() and r.start.service_is_ready() and r.manager.vehicle_client.service_is_ready())
    r.send(mission)
    client=r.helper.create_client(SetPlannerMode,'/pm/planner/execution_manager/set_perception_mode')
    pub=r.helper.create_publisher(TerrainAssessment,'/depth_elevation_mapper/planner_costmap',1)
    r.until(client.service_is_ready)
    def mode(value):
        req=SetPlannerMode.Request();req.mode=value
        f=client.call_async(req);r.until(f.done);return f.result()
    def publish(obstacle,terrain):
        msg=TerrainAssessment();msg.schema_version=1;msg.header.frame_id='map'
        msg.header.stamp=r.manager.get_clock().now().to_msg()
        msg.info.width=120;msg.info.height=120;msg.info.resolution=.05
        msg.info.origin.position.x=-3.;msg.info.origin.position.y=-3.;msg.info.origin.orientation.w=1.
        msg.obstacle_cost=[obstacle]*14400;msg.terrain_cost=[terrain]*14400
        pub.publish(msg);r.pump(80);return msg
    assert r.manager.perception_mode=='off'
    assert not mode('terrain_without_obstacles').success
    assert mode('obstacle_only').success
    assert 'costmap' in r.manager.readiness()
    assert not r.manager.start(Trigger.Request(),Trigger.Response()).success
    # terrainだけが100でも、obstacleのみではその値を混ぜない。
    msg=publish(0,100)
    assert r.manager.terrain.sample(0.,0.)==0
    r.begin();assert r.command[0]>0.
    assert mode('full_terrain').success
    assert r.manager.state=='DISABLED' and r.command==(0.,0.)
    assert r.manager.terrain is None
    publish(0,100)
    assert not r.manager.start(Trigger.Request(),Trigger.Response()).success
    assert 'footprint' in r.manager.readiness()
    # 総合の0で障害物unknownを隠さない。
    r.feed();publish(-1,0)
    assert r.manager.terrain.sample(0.,0.)==100
    assert not r.manager.start(Trigger.Request(),Trigger.Response()).success
    r.feed();publish(0,0);r.begin()
    # 同じmodeの再要求は走行を不用意に止めない。
    assert mode('full_terrain').success and r.manager.state=='RUNNING'
    assert mode('off').success
    assert r.manager.state=='DISABLED' and not r.manager.p['perception_enabled']
    r.begin();assert r.manager.state=='RUNNING'  # 明示回答どおりOFFでも経路追従可能


def test_assessment_rejects_invalid_or_old_and_does_not_refresh_duplicate(rig):
    from pm_msgs.msg import TerrainAssessment
    from pm_msgs.srv import SetPlannerMode
    r=rig;mission=load_document(r.ws/'src/pm_planner/examples/demo_mission.yaml')
    r.until(lambda:r.upload.service_is_ready() and r.start.service_is_ready() and r.manager.vehicle_client.service_is_ready())
    r.send(mission)
    r.manager.set_perception_mode(SetPlannerMode.Request(mode='obstacle_only'),SetPlannerMode.Response())
    msg=TerrainAssessment();msg.schema_version=1;msg.header.frame_id='map'
    msg.header.stamp=r.manager.get_clock().now().to_msg()
    msg.info.width=120;msg.info.height=120;msg.info.resolution=.05
    msg.info.origin.position.x=-3.;msg.info.origin.position.y=-3.;msg.info.origin.orientation.w=1.
    msg.obstacle_cost=[0]*14400
    r.manager.on_assessment(msg);assert r.manager.terrain is not None
    received=r.manager.terrain_received
    r.manager.on_assessment(msg)
    assert r.manager.terrain_received==received and r.manager.assessment_drop_count==1
    r.feed();msg.header.stamp=r.manager.get_clock().now().to_msg();msg.schema_version=9
    r.manager.on_assessment(msg);assert r.manager.terrain is None
    assert 'schema' in r.manager.terrain_error


def prepare_blocked_wait_case(r):
    """健全な専用mapの前提を用意する。mapの配送自体は別のDDS結合試験で検証済み。"""
    from nav_msgs.msg import OccupancyGrid
    r.until(lambda:r.upload.service_is_ready() and r.start.service_is_ready() and r.manager.vehicle_client.service_is_ready())
    r.send(load_document(r.ws/'src/pm_planner/examples/demo_mission.yaml'))
    r.manager.set_perception_mode(SetPlannerMode.Request(mode='obstacle_only'),SetPlannerMode.Response())
    def terrain(value):
        msg=OccupancyGrid();msg.header.frame_id='map';msg.header.stamp=r.manager.get_clock().now().to_msg()
        msg.info.width=120;msg.info.height=120;msg.info.resolution=.05
        msg.info.origin.position.x=-3.;msg.info.origin.position.y=-3.;msg.info.origin.orientation.w=1.
        msg.data=[value]*14400;r.manager.install_terrain(msg)
    terrain(0);r.begin()
    return terrain


@pytest.mark.parametrize('blocker',['footprint','trajectory'])
def test_blocked_wait_keeps_permission_and_recovers_without_start(rig,monkeypatch,blocker):
    from pm_planner.mppi_poc import NoValidTrajectoryError
    r=rig;terrain=prepare_blocked_wait_case(r)
    assert r.manager.get_parameter('keep_autonomy_on_blocked').value is True
    original=r.manager.follower.solver.step
    def blocked(*args,**kwargs):raise NoValidTrajectoryError('test: 有効候補なし')
    if blocker=='trajectory':monkeypatch.setattr(r.manager.follower.solver,'step',blocked)
    else:terrain(100)
    calls=list(r.calls);r.manager.last_permission_time-=2.;r.feed()
    assert r.manager.state=='WAITING' and r.command==(0.,0.) and r.permission
    for _ in range(20):
        terrain(100 if blocker=='footprint' else 0);r.feed()
        assert r.manager.state=='WAITING' and r.command==(0.,0.) and r.permission
    assert r.calls==calls  # 長時間待機でも許可false/trueの再要求はしない
    monkeypatch.setattr(r.manager.follower.solver,'step',original)
    terrain(0);r.v=.05;r.w=.06;r.feed()
    assert r.manager.state=='WAITING' and r.command==(0.,0.)
    r.v=r.w=0.;terrain(0);r.feed()
    assert r.manager.state=='RUNNING' and r.command[0]>0. and r.permission


@pytest.mark.parametrize('blocker',['footprint','trajectory'])
def test_disable_wait_option_retains_explicit_restart(rig,monkeypatch,blocker):
    from pm_planner.mppi_poc import NoValidTrajectoryError
    r=rig;r.manager.p['keep_autonomy_on_blocked']=False
    terrain=prepare_blocked_wait_case(r)
    if blocker=='footprint':terrain(100)
    else:
        def blocked(*args,**kwargs):raise NoValidTrajectoryError('test: 有効候補なし')
        monkeypatch.setattr(r.manager.follower.solver,'step',blocked)
    r.feed();r.until(lambda:not r.permission)
    assert r.manager.state==('DISABLED' if blocker=='footprint' else 'FAULT')
    terrain(0);r.feed();assert r.manager.state!='RUNNING' and r.command==(0.,0.)


@pytest.mark.parametrize('interruption',['stop','joy','pose','map_timeout','invalid_map','vehicle','gui','emergency','mode','watchdog'])
def test_waiting_still_disables_on_safety_or_manual_interruption(rig,interruption):
    r=rig;terrain=prepare_blocked_wait_case(r);terrain(100);r.feed()
    assert r.manager.state=='WAITING'
    if interruption=='stop':r.manager.stop(Trigger.Request(),Trigger.Response())
    elif interruption=='joy':r.joy.publish(Twist());r.pump()
    elif interruption=='pose':
        for _ in range(7):terrain(100);r.feed(pose=False)
    elif interruption=='map_timeout':
        for _ in range(12):r.feed()
    elif interruption=='invalid_map':
        from pm_msgs.msg import TerrainAssessment
        r.manager.on_assessment(TerrainAssessment(schema_version=99))
    elif interruption=='vehicle':
        r.manager.on_vehicle(String(data=json.dumps(dict(connected=True,armed=True,emergency_stop=False,autonomy_enabled=False))))
    elif interruption=='gui':
        r.manager.operator_received=time.monotonic()-2.;r.manager.tick()
    elif interruption=='emergency':
        from std_msgs.msg import Bool
        r.manager.on_emergency(Bool(data=True))
    elif interruption=='mode':
        r.manager.set_perception_mode(SetPlannerMode.Request(mode='off'),SetPlannerMode.Response())
    else:
        r.manager.last_command_time=time.monotonic()-2.;r.manager.tick()
    r.feed()
    assert r.manager.state=='DISABLED' and r.command==(0.,0.)
    r.until(lambda:not r.permission)
    terrain(0);r.feed();assert r.manager.state=='DISABLED'


def test_mapper_margin_parameter_reaches_environment(tmp_path):
    """ROSの起動parameterを上書きし、実際のcostmap膨張へ渡ることを確認する。"""
    if os.environ.get('ROS_DOMAIN_ID')!='213':pytest.skip('隔離DDS domainで実行する')
    rclpy.init(args=[]);r=Rig(tmp_path,[Parameter('footprint_margin_m',value=.23)])
    try:
        prepare_blocked_wait_case(r)
        assert r.manager.get_parameter('footprint_margin_m').value==pytest.approx(.23)
        assert r.manager.terrain.margin_m==pytest.approx(.23)
        assert r.manager.terrain.inflation_radius==pytest.approx(r.manager.p['footprint_radius_m']+.23)
    finally:r.close();rclpy.shutdown()
