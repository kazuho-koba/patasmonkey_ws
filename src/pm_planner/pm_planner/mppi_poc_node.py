"""参照PathとOdometryからMPPIのdebug指令を計算する非駆動ROS node。"""
from collections import deque
import json
import math
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from .io import read_limits
from .mppi_poc import MppiConfig, MppiPoc


class MppiPocNode(Node):
    """実車topicへpublishしない。frame不一致・pose期限切れはゼロをdebug出力。"""
    def __init__(self,parameter_overrides=None):
        super().__init__('mppi_poc',namespace='/pm/planner',parameter_overrides=parameter_overrides)
        defaults=dict(path_topic='/pm/planner/preview/reference_path',odom_topic='/odometry/global',
            base_frame='base_link',vehicle_config=get_package_share_directory('pm_config')+'/config/vehicle_control.yaml',
            pose_timeout_sec=0.5,rate_hz=10.,batch_size=384,steps=25,iterations=2,seed=42,
            temperature=.4,sigma_v=.22,sigma_w=.65,target_speed=.15,lookahead_m=.05,
            path_cost_weight=4.,heading_cost_weight=.2,speed_cost_weight=.5,yaw_cost_weight=.015,terminal_cost_weight=5.,
            max_projection_advance=.35,max_path_error=1.,goal_radius=.10,yaw_response_gain=1.0,visualize=True)
        for key,value in defaults.items():self.declare_parameter(key,value)
        p={key:self.get_parameter(key).value for key in defaults};self.p=p
        if not math.isfinite(p['rate_hz']) or not 1.<=p['rate_hz']<=25. or not math.isfinite(p['pose_timeout_sec']) or p['pose_timeout_sec']<=0:
            raise ValueError('rate_hzは1〜25、pose_timeout_secは正値が必要です')
        limits=read_limits(p['vehicle_config'])
        config=MppiConfig(batch_size=p['batch_size'],steps=p['steps'],iterations=p['iterations'],seed=p['seed'],
            dt=1./p['rate_hz'],temperature=p['temperature'],sigma_v=p['sigma_v'],sigma_w=p['sigma_w'],
            target_speed=p['target_speed'],max_speed=limits.max_speed,max_yaw_rate=limits.max_yaw_rate,
            max_yaw_accel=limits.max_yaw_accel,min_radius=limits.radius,lookahead_m=p['lookahead_m'],
            max_projection_advance=p['max_projection_advance'],max_path_error=p['max_path_error'],goal_radius=p['goal_radius'],
            path_cost_weight=p['path_cost_weight'],heading_cost_weight=p['heading_cost_weight'],
            speed_cost_weight=p['speed_cost_weight'],yaw_cost_weight=p['yaw_cost_weight'],
            terminal_cost_weight=p['terminal_cost_weight'],yaw_response_gain=p['yaw_response_gain'])
        self.solver=MppiPoc(config);self.frame='';self.odom=None;self.odom_time=None;self.timings=deque(maxlen=300)
        self.previous_command=(0.,0.)
        self.command_pub=self.create_publisher(Twist,'~/debug/cmd_vel',10)
        self.path_pub=self.create_publisher(Path,'~/debug/trajectory',10)
        self.status_pub=self.create_publisher(String,'~/debug/status',10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Path,p['path_topic'],self.on_path,qos)
        self.create_subscription(Odometry,p['odom_topic'],self.on_odom,qos_profile_sensor_data)
        self.create_timer(1./p['rate_hz'],self.tick)

    def on_path(self,msg):
        """Path更新ごとにoptimizerの履歴を破棄。空Pathは明示的な無効化。"""
        try:
            if not msg.header.frame_id:raise ValueError('Path frameが空です')
            if any(p.header.frame_id and p.header.frame_id!=msg.header.frame_id for p in msg.poses):
                raise ValueError('Path poseとheaderのframeが不一致')
            self.solver.set_path([(p.pose.position.x,p.pose.position.y) for p in msg.poses])
            self.frame=msg.header.frame_id
        except ValueError as exc:
            # 空Pathで旧予測を消すため、frameは可能なら以前の有効値を維持する。
            self.solver.path=None;self.frame=msg.header.frame_id or self.frame
            self.get_logger().warn('Path無効: '+str(exc))

    def on_odom(self,msg):
        self.odom=msg;self.odom_time=time.monotonic()

    def tick(self):
        """同一frame・新しいOdometryだけを使用し、例外時はゼロで結果を無効化。"""
        started=time.perf_counter();cmd=Twist();reason='';result=None
        try:
            if self.solver.path is None:raise ValueError('Path未受信')
            if self.odom is None or self.odom_time is None:raise ValueError('Odometry未受信')
            msg=self.odom;stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            age=self.get_clock().now().nanoseconds/1e9-stamp
            if not -.1<=age<=self.p['pose_timeout_sec'] or time.monotonic()-self.odom_time>self.p['pose_timeout_sec']:
                raise ValueError('Odometryが期限切れ')
            if msg.header.frame_id!=self.frame or msg.child_frame_id!=self.p['base_frame']:
                raise ValueError('Path/Odometry/base frame不一致。TF変換はPoC未対応')
            q=msg.pose.pose.orientation
            if not all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w)) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.02:
                raise ValueError('Odometry姿勢が無効')
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            position=msg.pose.pose.position;tw=msg.twist.twist
            result=self.solver.step((position.x,position.y,yaw),(tw.linear.x,tw.angular.z),
                                    previous_command=self.previous_command)
            cmd.linear.x,cmd.angular.z=result['command']
            self.previous_command=result['command']
        except (ValueError,ArithmeticError) as exc:
            reason=str(exc);self.solver.nominal.fill(0.);self.previous_command=(0.,0.)
        elapsed=(time.perf_counter()-started)*1000.;self.timings.append(elapsed)
        self.command_pub.publish(cmd)
        status=dict(valid=result is not None,reason=reason,stage='NON_DRIVING_POC',environment_checked=False,
                    station_m=result['station'] if result else None,goal=result['goal'] if result else False,
                    compute_ms=elapsed,p95_ms=float(np.percentile(self.timings,95)),
                    max_ms=max(self.timings),deadline_ms=1000./self.p['rate_hz'],
                    deadline_missed=elapsed>1000./self.p['rate_hz'])
        self.status_pub.publish(String(data=json.dumps(status,ensure_ascii=False)))
        if self.p['visualize'] and self.frame:
            path=Path();path.header.frame_id=self.frame;path.header.stamp=self.get_clock().now().to_msg()
            if result is not None:
                for x,y,yaw in result['trajectory']:
                    pose=PoseStamped();pose.header=path.header;pose.pose.position.x=float(x);pose.pose.position.y=float(y)
                    pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2);path.poses.append(pose)
            self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args);node=None;executor=SingleThreadedExecutor()
    try:
        node=MppiPocNode();executor.add_node(node);executor.spin()
    except KeyboardInterrupt:pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        executor.shutdown()
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
