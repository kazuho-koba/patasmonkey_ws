"""Stage 1のROS接続。Path・Marker・状態だけをpublishし、Twistは扱わない。

manual anchorまたは既存FromLLの明示frame契約を利用する。サービスは非同期で
処理し、TFは観測stampでlookupする。datumや既存TFを設定／発行しない。
"""
from dataclasses import asdict
import hashlib
import json
import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.clock import Clock,ClockType
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Path,Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker,MarkerArray
from robot_localization.srv import FromLL
from tf2_ros import Buffer,TransformListener,TransformException
from ament_index_python.packages import get_package_share_directory

from .geodesy import planar_transform
from .io import read_mission,read_anchor,read_limits
from .reference import build_reference
from .progress import Progress
from pm_mission.model import validate


def stamp_seconds(stamp):return stamp.sec+stamp.nanosec/1e9


def yaw_from_quaternion(q):
    """無効姿勢をheading=0へ置換せず拒否する。"""
    values=(q.x,q.y,q.z,q.w)
    if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values)-1)>.02:
        raise ValueError('pose quaternionが無効です')
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class PreviewNode(Node):
    def __init__(self,parameter_overrides=None):
        super().__init__('preview',namespace='/pm/planner',parameter_overrides=parameter_overrides)
        defaults=dict(mission_file='',managed_mission_enabled=False,coordinate_mode='navsat',anchor_file='',planning_frame='map',
            navsat_world_frame='',navsat_service='/fromLL',navsat_odom_topic='/odometry/gps',gnss_topic='/fix/gated',
            pose_topic='/odometry/local',base_frame='base_link',pose_timeout_sec=.5,gnss_timeout_sec=2.5,
            service_timeout_sec=3.,max_projection_tilt_rad=.05,spacing_m=.1,corner_allowance_m=.3,
            sharp_turn_strategy='symmetric_bulb',uturn_direction='counterclockwise',
            pass_radius_m=.5,stop_radius_m=.25,start_radius_m=.5,max_deviation_m=1.,max_advance_m=2.,
            max_pose_gap_sec=1.,jump_slack_m=.3,stop_speed_mps=.03,heading_tolerance_rad=.52,max_segment_m=500.,
            vehicle_config=get_package_share_directory('pm_config')+'/config/vehicle_control.yaml')
        for key,value in defaults.items():
            self.declare_parameter(key,value,ParameterDescriptor(read_only=key!='mission_file'))
        self.p={key:self.get_parameter(key).value for key in defaults}
        for key in ('pose_timeout_sec','gnss_timeout_sec','service_timeout_sec','max_projection_tilt_rad'):
            if not math.isfinite(self.p[key]) or self.p[key]<=0:raise ValueError(key+'は正の有限値が必要です')
        self.limits=read_limits(self.p['vehicle_config'])
        self.frame=self.p['planning_frame'];self.mission=None;self.reference=None;self.progress=None
        self.points=[];self.pose=None;self.pose_received=None;self.pose_error='pose未受信'
        self.fix=None;self.gps=None;self.load_state='IDLE';self.load_error='';self.provenance={}
        self.future=None;self.generation=0;self.waiting=False;self.convert_index=0
        self.buffer=Buffer(cache_time=Duration(seconds=30.));self.listener=TransformListener(self.buffer,self)
        latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub=self.create_publisher(Path,'~/reference_path',latched)
        self.polyline_pub=self.create_publisher(Path,'~/mission_polyline',latched)
        self.marker_pub=self.create_publisher(MarkerArray,'~/markers',latched)
        self.progress_marker_pub=self.create_publisher(MarkerArray,'~/progress_markers',latched)
        self.status_pub=self.create_publisher(String,'~/status',10)
        if self.p['managed_mission_enabled']:
            self.create_subscription(String, '/pm/planner/execution_manager/mission', self.on_managed_mission, latched)
        self.create_subscription(Odometry,self.p['pose_topic'],self.on_pose,qos_profile_sensor_data)
        self.create_subscription(Odometry,self.p['navsat_odom_topic'],self.on_gps,qos_profile_sensor_data)
        self.create_subscription(NavSatFix,self.p['gnss_topic'],self.on_fix,qos_profile_sensor_data)
        self.client=self.create_client(FromLL,self.p['navsat_service'])
        for name,callback in (('load',self.on_load),('start',self.on_start),('pause',self.on_pause),
                              ('resume',self.on_resume),('confirm',self.on_confirm),('cancel',self.on_cancel),('reset',self.on_reset)):
            self.create_service(Trigger,'~/'+name,callback)
        # ROS /clockが停止してもfreshnessを延長せず、状態通知を継続する。
        self.initial_load=True
        self.timer=self.create_timer(.2,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Stage 1 previewのみ。速度・motor出力、TF発行はありません。')

    def fresh(self,message,received,timeout):
        if message is None or received is None:return False
        age=self.get_clock().now().nanoseconds/1e9-stamp_seconds(message.header.stamp)
        return -.1<=age<=timeout and time.monotonic()-received<=timeout

    def on_fix(self,message):self.fix=(message,time.monotonic())
    def on_gps(self,message):self.gps=(message,time.monotonic())

    def on_pose(self,message):
        """同stampのTFでplanning frameへ。古いposeは追跡・startに使わない。"""
        try:
            now=time.monotonic()
            if not self.fresh(message,now,self.p['pose_timeout_sec']):raise ValueError('pose stampが古い／未来です')
            if message.child_frame_id!=self.p['base_frame']:raise ValueError('Odometry child frameがbase_frameと不一致')
            position=message.pose.pose.position;yaw=yaw_from_quaternion(message.pose.pose.orientation)
            x,y=position.x,position.y
            if not message.header.frame_id:raise ValueError('pose frameが空です')
            if message.header.frame_id!=self.frame:
                if stamp_seconds(message.header.stamp)<=0:raise ValueError('TFに使うpose stampが0です')
                transform=self.buffer.lookup_transform(self.frame,message.header.frame_id,Time.from_msg(message.header.stamp)).transform
                x,y=planar_transform(transform,x,y,position.z,self.p['max_projection_tilt_rad'])
                yaw+=yaw_from_quaternion(transform.rotation)
            v=message.twist.twist.linear;speed=math.hypot(v.x,v.y)
            pose=(stamp_seconds(message.header.stamp),x,y,yaw,speed)
            if not all(math.isfinite(a) for a in pose):raise ValueError('pose／速度が非有限です')
            self.pose,self.pose_received,self.pose_error=pose,now,''
            if self.progress:self.progress.update(pose)
        except (ValueError,TransformException) as exc:
            self.pose_error=str(exc);self.pose=None;self.pose_received=None
            if self.progress:self.progress.pause(self.pose_error)

    def require_pose(self):
        if self.pose is None or self.pose_received is None:raise ValueError(self.pose_error or 'pose未受信')
        age=self.get_clock().now().nanoseconds/1e9-self.pose[0]
        if not (-.1<=age<=self.p['pose_timeout_sec']) or time.monotonic()-self.pose_received>self.p['pose_timeout_sec']:
            raise ValueError('poseが期限切れです')
        return self.pose

    def on_managed_mission(self,message):
        """実行管理の検証済みsnapshotを再検証し、旧Pathを無効化して変換する。"""
        try:
            payload=json.loads(message.data)
            mission=validate(payload['mission'],for_submission=True)
            digest=hashlib.sha256(json.dumps(mission,sort_keys=True).encode()).hexdigest()
            if digest!=payload['sha256']:raise ValueError('mission hashが一致しません')
            if self.progress:self.progress.pause('新規missionを受領')
            self.load(mission)
        except (ValueError,TypeError,KeyError) as exc:
            self.generation+=1;self.waiting=False;self.future=None
            self.mission=None;self.points=[];self.reference=None;self.progress=None
            self.clear_geometry();self.fail_load('実行管理からのmissionが不正: '+str(exc))

    def load(self,mission_document=None):
        """元fileは変更しない。reloadでは旧Path／進捗を先に無効化する。"""
        if self.progress and self.progress.state in ('RUNNING','STOP_REQUIRED','WAITING_PAUSE','WAITING_CONFIRM'):
            raise ValueError('追跡中はloadできません。pauseまたはresetしてください')
        self.generation+=1;self.future=None;self.waiting=False;self.progress=None;self.reference=None
        self.mission=None;self.points=[];self.pose=None;self.pose_received=None;self.provenance={}
        self.load_state='LOADING';self.load_error=''
        self.clear_geometry()
        try:
            if mission_document is None:
                path=self.get_parameter('mission_file').value
                self.mission=read_mission(path)
            else:
                self.mission=validate(mission_document,for_submission=True)
            self.provenance['mission_sha256']=hashlib.sha256(json.dumps(self.mission,sort_keys=True).encode()).hexdigest()
            if self.p['coordinate_mode']=='manual':
                anchor=read_anchor(self.p['anchor_file']);self.frame=anchor.frame_id
                self.provenance.update(mode='manual',anchor=asdict(anchor))
                self.points=[anchor.project(w['latitude'],w['longitude']) for w in self.mission['waypoints']]
                self.finish_load()
            elif self.p['coordinate_mode']=='navsat':
                self.frame=self.p['planning_frame'];self.waiting=True
                self.load_state='WAITING_FOR_NAVSAT'
            else:raise ValueError('coordinate_modeはmanualまたはnavsatです')
        except (ValueError,OSError,KeyError,TypeError) as exc:self.fail_load(exc)

    def fail_load(self,error):
        self.load_state='ERROR';self.load_error=str(error);self.waiting=False;self.future=None
        self.reference=None;self.progress=None
        self.path_pub.publish(self.path_message([]))
        self.get_logger().error(self.load_error)

    def begin_navsat(self):
        """サービス名でなく、明示したworld frameと新しいgps odometryで準備確認。"""
        source=self.p['navsat_world_frame']
        if not source:self.load_error='navsat_world_frameを実際の出力frameに合わせて明示してください';return
        if not self.gps or not self.fix:
            self.load_error='GNSSまたはnavsat odometryを待っています';return
        gps,gt=self.gps;fix,ft=self.fix
        timeout=self.p['gnss_timeout_sec']
        if not self.fresh(gps,gt,timeout) or not self.fresh(fix,ft,timeout):
            self.load_error='GNSSまたはnavsat odometryが期限切れです';return
        if gps.header.frame_id!=source:
            self.load_error='navsat odometry frameと明示navsat_world_frameが不一致';return
        if fix.status.status<0 or not all(math.isfinite(v) for v in (fix.latitude,fix.longitude,fix.altitude)):
            self.load_error='GNSS fixまたは高度が無効です';return
        if not self.client.service_is_ready():
            self.load_error='FromLLサービスを待っています';return
        transform=None
        if source!=self.frame:
            # TF2では時刻0がlatestを意味するため、未設定stampでの変換は行わない。
            if stamp_seconds(gps.header.stamp)<=0:self.load_error='TFに使うGNSS stampが0です';return
            try:transform=self.buffer.lookup_transform(self.frame,source,Time.from_msg(gps.header.stamp)).transform
            except TransformException as exc:self.load_error=str(exc);return
            try:planar_transform(transform,0.,0.,0.,self.p['max_projection_tilt_rad'])
            except ValueError as exc:self.load_error=str(exc);return
        self.projection_transform=transform;self.projection_altitude=fix.altitude
        self.provenance.update(mode='navsat',source_frame=source,planning_frame=self.frame,
            conversion_stamp_ns=int(stamp_seconds(gps.header.stamp)*1e9),altitude_m=fix.altitude,
            alignment_verified=False,note='frame契約を確認したpreview。地理的真値との整合は実測未検証')
        if transform:
            self.provenance['transform']={k:[getattr(getattr(transform,k),a) for a in ('x','y','z')+ (('w',) if k=='rotation' else ())] for k in ('translation','rotation')}
        self.waiting=False;self.load_error='';self.convert_index=0
        self.send_conversion()

    def send_conversion(self):
        if self.convert_index==len(self.mission['waypoints']):self.finish_load();return
        waypoint=self.mission['waypoints'][self.convert_index]
        request=FromLL.Request();request.ll_point.latitude=waypoint['latitude']
        request.ll_point.longitude=waypoint['longitude'];request.ll_point.altitude=self.projection_altitude
        generation=self.generation;self.future=self.client.call_async(request)
        self.future_started=time.monotonic();self.load_state='CONVERTING'
        self.future.add_done_callback(lambda future:self.on_conversion(future,generation))

    def on_conversion(self,future,generation):
        if generation!=self.generation or future is not self.future:return
        try:
            point=future.result().map_point
            if not all(math.isfinite(v) for v in (point.x,point.y,point.z)):raise ValueError('FromLL応答が非有限です')
            xy=(point.x,point.y)
            if self.projection_transform:
                xy=planar_transform(self.projection_transform,point.x,point.y,point.z,self.p['max_projection_tilt_rad'])
            self.points.append(xy);self.convert_index+=1;self.future=None;self.send_conversion()
        except (ValueError,RuntimeError,AttributeError) as exc:self.fail_load(exc)

    def finish_load(self):
        self.polyline_pub.publish(self.path_message([(x,y,0.,0.) for x,y in self.points]))
        try:
            self.reference=build_reference(self.points,self.mission['waypoints'],self.limits,
                self.p['spacing_m'],self.p['corner_allowance_m'],self.p['pass_radius_m'],self.p['max_segment_m'],
                sharp_turn_strategy=self.p['sharp_turn_strategy'],uturn_direction=self.p['uturn_direction'])
            self.progress=Progress(self.reference,self.p['pass_radius_m'],self.p['stop_radius_m'],self.p['start_radius_m'],
                self.p['max_deviation_m'],self.p['max_advance_m'],self.p['jump_slack_m'],self.p['max_pose_gap_sec'],
                self.p['stop_speed_mps'],self.p['heading_tolerance_rad'])
            self.load_state='READY';self.path_pub.publish(self.path_message(self.reference.samples))
        except ValueError as exc:self.fail_load(exc)
        self.publish_waypoints()

    def path_message(self,samples):
        message=Path();message.header.frame_id=self.frame;message.header.stamp=self.get_clock().now().to_msg()
        for x,y,yaw,_ in samples:
            pose=PoseStamped();pose.header=message.header;pose.pose.position.x=float(x);pose.pose.position.y=float(y)
            pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2);message.poses.append(pose)
        return message

    def clear_geometry(self):
        self.path_pub.publish(self.path_message([]));self.polyline_pub.publish(self.path_message([]))
        marker=Marker();marker.action=Marker.DELETEALL;marker.header.frame_id=self.frame
        self.marker_pub.publish(MarkerArray(markers=[marker]))
        self.progress_marker_pub.publish(MarkerArray(markers=[marker]))

    def publish_waypoints(self):
        """大量のtext markerを毎周期複製しない。先頭200点までをload時に発行。"""
        markers=[]
        bulb_turns={t['waypoint_index']:t for t in self.reference.turns
                    if t['mode']=='symmetric_bulb'} if self.reference else {}
        for i,(point,waypoint) in enumerate(zip(self.points[:200],self.mission['waypoints'][:200])):
            marker=Marker();marker.header.frame_id=self.frame;marker.header.stamp=self.get_clock().now().to_msg()
            marker.ns='waypoints';marker.id=i;marker.type=Marker.TEXT_VIEW_FACING;marker.action=Marker.ADD
            marker.pose.position.x=float(point[0]);marker.pose.position.y=float(point[1]);marker.pose.position.z=.15
            marker.pose.orientation.w=1.;marker.scale.z=.18;marker.color.r=.8;marker.color.a=1.
            marker.text='%d %s'%(i+1,waypoint['attribute'])
            if i in bulb_turns:
                turn=bulb_turns[i]
                marker.text+=' / bulb %s R=%.2fm'%(
                    'CCW' if turn['direction']=='counterclockwise' else 'CW',turn['radius_m'])
            markers.append(marker)
        self.marker_pub.publish(MarkerArray(markers=markers))

    def tick(self):
        if self.initial_load:
            self.initial_load=False
            if not self.p['managed_mission_enabled'] and self.get_parameter('mission_file').value:self.load()
        if self.waiting:self.begin_navsat()
        if self.future and time.monotonic()-self.future_started>self.p['service_timeout_sec']:
            self.fail_load('FromLL service timeout')
        try:self.require_pose()
        except ValueError as exc:
            self.pose_error=str(exc)
            if self.progress:self.progress.pause(self.pose_error)
        status=dict(stage=1,dry_run=True,drive_ready=False,velocity_publisher=False,
            load_state=self.load_state,load_error=self.load_error,frame_id=self.frame,
            pose_error=self.pose_error,pose_fresh=not bool(self.pose_error),coordinate_provenance=self.provenance,
            vehicle_limits=asdict(self.limits),geometry_valid=self.reference is not None,
            turn_policy=self.p['sharp_turn_strategy'],turns=self.reference.turns if self.reference else [],
            environment_checked=False,
            mission_id=self.mission['mission_id'] if self.mission else None,
            revision=self.mission['revision'] if self.mission else None,
            progress=self.progress.status() if self.progress else None)
        self.status_pub.publish(String(data=json.dumps(status,ensure_ascii=False,allow_nan=False)))
        markers=[]
        # 静的なwaypointラベルとは別topicにし、late joinでも両方を受信できる。
        if self.progress and self.progress.index<len(self.reference.events):
            event=self.reference.events[self.progress.index]
            marker=Marker();marker.header.frame_id=self.frame;marker.header.stamp=self.get_clock().now().to_msg()
            marker.ns='target';marker.id=0;marker.type=Marker.SPHERE;marker.pose.orientation.w=1.
            marker.pose.position.x=float(event['x']);marker.pose.position.y=float(event['y'])
            marker.scale.x=marker.scale.y=marker.scale.z=.3;marker.color.r=1.;marker.color.b=.6;marker.color.a=.6
            markers.append(marker)
        else:
            marker=Marker();marker.ns='target';marker.id=0;marker.action=Marker.DELETE;markers.append(marker)
        if self.pose:
            marker=Marker();marker.header.frame_id=self.frame;marker.header.stamp=self.get_clock().now().to_msg()
            marker.ns='pose';marker.id=1;marker.type=Marker.ARROW
            marker.pose.position.x=float(self.pose[1]);marker.pose.position.y=float(self.pose[2])
            marker.pose.orientation.z=math.sin(self.pose[3]/2);marker.pose.orientation.w=math.cos(self.pose[3]/2)
            marker.scale.x=.6;marker.scale.y=marker.scale.z=.08;marker.color.g=.7;marker.color.b=1.;marker.color.a=1.
            markers.append(marker)
        else:
            marker=Marker();marker.ns='pose';marker.id=1;marker.action=Marker.DELETE;markers.append(marker)
        self.progress_marker_pub.publish(MarkerArray(markers=markers))

    def command(self,response,callback):
        try:
            callback();response.success=True;response.message='preview状態を更新しました。車体は駆動しません。'
        except (ValueError,AttributeError) as exc:response.success=False;response.message=str(exc)
        return response

    def on_load(self,request,response):
        if self.p['managed_mission_enabled']:
            response.success=False;response.message='実行管理接続時は受領missionから自動loadします';return response
        try:self.load()
        except ValueError as exc:response.success=False;response.message=str(exc);return response
        response.success=self.load_state!='ERROR'
        response.message='load受付。READY／ERRORの結果はstatusで確認してください。'+self.load_error
        return response

    def on_start(self,request,response):
        return self.command(response,lambda:self.progress.start(self.require_pose()))

    def on_pause(self,request,response):
        return self.command(response,lambda:self.progress.pause())

    def on_resume(self,request,response):
        return self.command(response,lambda:self.progress.resume(self.require_pose()))

    def on_confirm(self,request,response):
        def confirm():self.require_pose();self.progress.confirm()
        return self.command(response,confirm)

    def on_cancel(self,request,response):
        return self.command(response,lambda:self.progress.cancel())

    def on_reset(self,request,response):
        def reset():
            if self.progress:self.progress.pause('reset')
            self.load(self.mission if self.p['managed_mission_enabled'] else None)
        return self.command(response,reset)


def main(args=None):
    rclpy.init(args=args);node=None;executor=SingleThreadedExecutor()
    try:
        node=PreviewNode();executor.add_node(node);executor.spin()
    except KeyboardInterrupt:pass
    finally:
        # terminalのprocess groupとlaunchの両方からSIGINTが来ても、購読破棄を
        # 二度目のKeyboardInterruptで中断しない。専用executorも先に明示終了する。
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        executor.shutdown()
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
