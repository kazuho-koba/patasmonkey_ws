"""MISSION YAMLを使う独立した非駆動MPPI閉ループ可視化。"""
from collections import deque
from copy import deepcopy
from dataclasses import asdict
import json
import math
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory

from .io import read_anchor, read_limits, read_mission
from .mppi_poc import MppiConfig, MppiPoc
from .progress import Progress
from .reference import build_reference


class MppiLiveDemo(Node):
    """車体を模擬し専用topicだけを発行。実車・既存localizationへ一切接続しない。"""
    def __init__(self, parameter_overrides=None):
        super().__init__('mppi_demo', namespace='/pm/planner', parameter_overrides=parameter_overrides)
        planner_share=get_package_share_directory('pm_planner')
        defaults=dict(mission_file=get_package_share_directory('pm_mission')+'/missions/home.yaml',
            anchor_file=planner_share+'/config/home_preview_anchor.yaml',
            vehicle_config=get_package_share_directory('pm_config')+'/config/vehicle_control.yaml',
            time_scale=1.0, candidate_sample_count=16, candidate_publish_hz=2.0,
            track_spacing_m=0.15, track_max_points=3000, vehicle_length_m=0.65, vehicle_width_m=0.45,
            sim_yaw_response_gain=1.0, reference_turn_margin_m=0.1)
        for key,value in defaults.items():self.declare_parameter(key,value)
        settings=asdict(MppiConfig())
        for key in ('max_speed','max_yaw_rate','max_yaw_accel','min_radius','candidate_sample_count'):
            settings.pop(key)
        for key,value in settings.items():self.declare_parameter(key,value)
        self.p={key:self.get_parameter(key).value for key in defaults}
        p=self.p
        if not math.isfinite(p['sim_yaw_response_gain']) or not 0.0 < p['sim_yaw_response_gain'] <= 2.0:
            raise ValueError('sim_yaw_response_gainは0より大きく2以下です')
        if not 0.1<=p['time_scale']<=10.0:raise ValueError('time_scaleは0.1〜10.0です')
        if not 0.<p['candidate_publish_hz']<=10. or not 0.<p['track_spacing_m']<=5.:
            raise ValueError('候補表示頻度または軌跡間隔が範囲外です')
        if not 2<=p['track_max_points']<=10000 or not 0.<p['vehicle_width_m']<=3. or not 0.<p['vehicle_length_m']<=5.:
            raise ValueError('軌跡点数または車体表示寸法が範囲外です')
        mission=read_mission(p['mission_file']);anchor=read_anchor(p['anchor_file']);limits=read_limits(p['vehicle_config'])
        # 中間停止属性はPoCで未対応。無視して走破せず、明示的に拒否する。
        if any(w['attribute'] not in ('start','pass','goal') for w in mission['waypoints']):
            raise ValueError('デモはPAUSE/CONFIRM属性に未対応です')
        if anchor.frame_id in ('map','odom','base_link','mppi_demo_base'):
            raise ValueError('表示専用anchorには実機と異なるframeを指定してください')
        points=[anchor.project(w['latitude'],w['longitude']) for w in mission['waypoints']]
        # 実車の制約はMPPI側に残し、参照線のみ追加の旋回余裕を設ける。
        self.reference=build_reference(points,mission['waypoints'],limits.with_turn_margin(p['reference_turn_margin_m']))
        self.frame=anchor.frame_id
        values={key:self.get_parameter(key).value for key in settings}
        self.config=MppiConfig(**values, max_speed=limits.max_speed, max_yaw_rate=limits.max_yaw_rate,
            max_yaw_accel=limits.max_yaw_accel, min_radius=limits.radius,
            candidate_sample_count=p['candidate_sample_count'])
        self.solver=MppiPoc(self.config)
        self.tf=TransformBroadcaster(self)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)
        self.reference_pub=self.create_publisher(Path,'~/reference_path',latched)
        self.track_pub=self.create_publisher(Path,'~/track',latched)
        self.prediction_pub=self.create_publisher(Path,'~/prediction',10)
        self.odom_pub=self.create_publisher(Odometry,'~/odometry',10)
        self.vehicle_pub=self.create_publisher(MarkerArray,'~/vehicle',10)
        self.candidates_pub=self.create_publisher(Marker,'~/candidates',10)
        self.status_pub=self.create_publisher(String,'~/status',10)
        self.create_service(Trigger,'~/pause',self.pause)
        self.create_service(Trigger,'~/resume',self.resume)
        self.create_service(Trigger,'~/reset',self.reset)
        self._reset()
        self.reference_pub.publish(self.path_message(self.reference.samples))
        # wall timer周期は dt/time_scale。過負荷時は高速化を維持できず実時間より遅れる。
        self.create_timer(self.config.dt/p['time_scale'],self.tick)
        self.get_logger().info('非駆動MPPIデモ: %s (frame=%s)'%(mission['name'],self.frame))

    def _reset(self):
        self.solver.set_path([(x,y) for x,y,_,_ in self.reference.samples])
        self.x,self.y,self.yaw=self.reference.samples[0][:3]
        self.v=self.w=self.sim_time=0.0
        self.progress=Progress(self.reference)
        self.progress.start((0.,self.x,self.y,self.yaw,0.))
        self.track=deque([(self.x,self.y)],maxlen=self.p['track_max_points'])
        self.paused=False;self.finished=False;self.reason='実行中';self.last_track_publish=0.;self.last_candidates_publish=0.
        self.timings=deque(maxlen=300);self.started_wall=time.monotonic();self.last_visual_publish=0.;self._publish_track()
        self.prediction_pub.publish(self.path_message([]))
        self._publish_candidates({})

    def pause(self,request,response):
        self.paused=True;self.reason='一時停止中';response.success=True;response.message=self.reason
        return response

    def resume(self,request,response):
        if self.finished:
            response.success=False;response.message='完了・異常後はresetが必要です';return response
        self.paused=False;self.reason='実行中';response.success=True;response.message=self.reason
        return response

    def reset(self,request,response):
        self._reset();response.success=True;response.message='人工車体を出発点へ戻しました'
        return response

    def path_message(self, samples):
        msg=Path();msg.header.frame_id=self.frame;msg.header.stamp=self.get_clock().now().to_msg()
        for sample in samples:
            pose=PoseStamped();pose.header=msg.header
            pose.pose.position.x=float(sample[0]);pose.pose.position.y=float(sample[1])
            yaw=float(sample[2]) if len(sample)>2 else 0.
            pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
            msg.poses.append(pose)
        return msg

    def _publish_track(self):
        self.track_pub.publish(self.path_message(self.track))

    def _publish_candidates(self,result):
        marker=Marker();marker.header.frame_id=self.frame;marker.header.stamp=self.get_clock().now().to_msg()
        marker.ns='samples';marker.id=0;marker.type=Marker.LINE_LIST;marker.action=Marker.ADD
        marker.pose.orientation.w=1.;marker.scale.x=.015;marker.color.r=.45;marker.color.g=.45;marker.color.b=.45;marker.color.a=.35
        candidates=result.get('candidate_trajectories')
        if candidates is not None:
            for candidate in candidates:
                for a,b in zip(candidate,candidate[1:]):
                    marker.points.extend([Point(x=float(a[0]),y=float(a[1]),z=.03),
                                          Point(x=float(b[0]),y=float(b[1]),z=.03)])
        self.candidates_pub.publish(marker)

    def tick(self):
        """solver指令を点車体へdt秒だけ適用し、観測・予測・表示を更新する。"""
        result=None;compute_ms=0.;now=time.monotonic()
        if not self.paused and not self.finished:
            started=time.perf_counter()
            try:
                result=self.solver.step((self.x,self.y,self.yaw),
                                        (self.v,self.w*self.p['sim_yaw_response_gain']),
                                        previous_command=(self.v,self.w))
                compute_ms=(time.perf_counter()-started)*1000.;self.timings.append(compute_ms)
                self.v,self.w=result['command']
                # 仮想車体側とMPPI予測側の係数は独立し、モデル不一致を試せる。
                self.yaw+=self.w*self.p['sim_yaw_response_gain']*self.config.dt
                self.x+=self.v*math.cos(self.yaw)*self.config.dt
                self.y+=self.v*math.sin(self.yaw)*self.config.dt
                self.sim_time+=self.config.dt
                self.progress.update((self.sim_time,self.x,self.y,self.yaw,self.v))
                if math.hypot(self.x-self.track[-1][0],self.y-self.track[-1][1])>=self.p['track_spacing_m']:
                    self.track.append((self.x,self.y))
                if result['goal'] or self.progress.state=='FAULT':
                    self.finished=True;self.v=self.w=0.
                    self.reason=('完了' if result['goal'] and self.progress.state=='COMPLETE' else
                                 '停止: 進捗判定='+self.progress.state+' '+self.progress.reason)
            except (ValueError,ArithmeticError) as exc:
                self.finished=True;self.v=self.w=0.;self.reason='計算停止: '+str(exc)
        # 演算の高速再生時も表示・TFの生成は実時間で最大10 Hzに抑える。
        if now-self.last_visual_publish < 0.1:
            return
        self.last_visual_publish=now
        stamp=self.get_clock().now().to_msg()
        transform=TransformStamped();transform.header.stamp=stamp;transform.header.frame_id=self.frame
        transform.child_frame_id='mppi_demo_base';transform.transform.translation.x=self.x;transform.transform.translation.y=self.y
        transform.transform.rotation.z=math.sin(self.yaw/2);transform.transform.rotation.w=math.cos(self.yaw/2)
        self.tf.sendTransform(transform)
        odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id=self.frame;odom.child_frame_id='mppi_demo_base'
        odom.pose.pose.position.x=self.x;odom.pose.pose.position.y=self.y;odom.pose.pose.orientation=transform.transform.rotation
        display_v=0.0 if self.paused else self.v
        display_w=0.0 if self.paused else self.w*self.p['sim_yaw_response_gain']
        odom.twist.twist.linear.x=display_v;odom.twist.twist.angular.z=display_w;self.odom_pub.publish(odom)
        body=Marker();body.header=odom.header;body.ns='body';body.id=0;body.type=Marker.CUBE
        body.pose=deepcopy(odom.pose.pose);body.pose.position.z=.12;body.scale.x=self.p['vehicle_length_m']
        body.scale.y=self.p['vehicle_width_m'];body.scale.z=.20;body.color.g=.7;body.color.b=.8;body.color.a=.75
        arrow=Marker();arrow.header=odom.header;arrow.ns='heading';arrow.id=1;arrow.type=Marker.ARROW
        arrow.pose=deepcopy(odom.pose.pose);arrow.pose.position.z=.28;arrow.scale.x=.55;arrow.scale.y=.08;arrow.scale.z=.08
        arrow.color.r=.1;arrow.color.g=.15;arrow.color.b=.2;arrow.color.a=1.
        label=Marker();label.header=odom.header;label.ns='status';label.id=2;label.type=Marker.TEXT_VIEW_FACING
        label.pose.position.x=self.x;label.pose.position.y=self.y;label.pose.position.z=.7;label.pose.orientation.w=1.
        label.scale.z=.22;label.color.r=.05;label.color.g=.05;label.color.b=.05;label.color.a=1.
        label.text='SIM  %.1f/%.1f m  v=%.2f m/s'%(self.solver.station,self.solver.s[-1],display_v)
        self.vehicle_pub.publish(MarkerArray(markers=[body,arrow,label]))
        if result is not None:
            self.prediction_pub.publish(self.path_message(result['trajectory']))
            if now-self.last_candidates_publish>=1./self.p['candidate_publish_hz']:
                self._publish_candidates(result);self.last_candidates_publish=now
        if now-self.last_track_publish>=1.0:
            self._publish_track();self.last_track_publish=now
        status=dict(stage='SIMULATION_ONLY',state=('PAUSED' if self.paused else 'FINISHED' if self.finished else 'RUNNING'),
            reason=self.reason,sim_time_s=self.sim_time,time_scale=self.p['time_scale'],
            station_m=self.solver.station,path_length_m=float(self.solver.s[-1]),
            progress=self.progress.status(),v_mps=display_v,w_radps=display_w,compute_ms=compute_ms,
            compute_p95_ms=float(np.percentile(self.timings,95)) if self.timings else None,
            candidate_sample_count=self.p['candidate_sample_count'],environment_checked=False,drive_output=False)
        self.status_pub.publish(String(data=json.dumps(status,ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args);node=None;executor=SingleThreadedExecutor()
    try:
        node=MppiLiveDemo();executor.add_node(node);executor.spin()
    except KeyboardInterrupt:pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);executor.shutdown()
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
