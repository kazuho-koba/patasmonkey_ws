"""実観測を使う非駆動MPPI preview。速度・車両許可のpublisherを持たない。

local EKF/composerの座標系で、最初の位置・向きから直進参照線を固定する。
予測を描くだけで車体をシミュレーション移動しない。poseはlocal推定だけが更新する。
"""
import math
import time
import json
import yaml
from pathlib import Path as FilePath

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from pm_msgs.msg import TerrainAssessment
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import ParameterDescriptor
from .mppi_poc import MppiConfig, MppiPoc
from .perception_input import select_costs
from .terrain_cost import TerrainCost
from .io import read_limits
from ament_index_python.packages import get_package_share_directory


FRAME='odom'
PREFIX='/perception_preview'


class PerceptionPreview(Node):
    def __init__(self, **kwargs):
        super().__init__('perception_preview', **kwargs)
        config=FilePath(get_package_share_directory('pm_config'))/'config/planner.yaml'
        shared=yaml.safe_load(config.read_text())['/**']['ros__parameters']
        # 表示用の速度・予測時間を別に作らない。通常plannerと同じ正本を読む。
        # 表示timerの2Hzとは独立に、内部予測のdtはcontroller_rate_hzから決める。
        self.declare_parameter('prediction_steps',int(shared['steps']),ParameterDescriptor(read_only=True))
        self.declare_parameter('prediction_dt',1./float(shared['controller_rate_hz']),ParameterDescriptor(read_only=True))
        # 車体半径と追加余裕も正本から取得し、実走行側と同じ和でcostを膨張する。
        # marginは既にm単位なので、セル幅を更に足すことはしない。
        defaults=dict(mode='obstacle_only', reference_length_m=8., target_speed=float(shared['target_speed']),
                      footprint_radius_m=float(shared['footprint_radius_m']),
                      footprint_margin_m=float(shared.get('footprint_margin_m',.1)), input_timeout_sec=1.0, preview_rate_hz=2.,
                      require_heading_ready=True, reference_mode='fixed_start')
        defaults.update({k:float(shared.get(k,getattr(MppiConfig(),k))) for k in
                         ('clearance_distance_m','clearance_cost_weight','clearance_min_speed_mps')})
        for name,value in defaults.items():self.declare_parameter(name,value)
        self.pose=None;self.pose_received=0.;self.assessment=None;self.map_received=0.
        self.velocity=(0.,0.)
        self.reference=None;self.solver=None;self.reset_reference=True;self.heading_ready=False
        self.cached_environment=None;self.cached_environment_key=None
        self.limits=read_limits(FilePath(get_package_share_directory('pm_config'))/'config/vehicle_control.yaml')
        self.map_pub=self.create_publisher(OccupancyGrid,PREFIX+'/selected_costmap',1)
        self.reference_pub=self.create_publisher(Path,PREFIX+'/reference_path',1)
        self.path_pub=self.create_publisher(Path,PREFIX+'/local_path',1)
        self.status_pub=self.create_publisher(String,PREFIX+'/status',1)
        self.speed_pub=self.create_publisher(String,PREFIX+'/speed_profile',1)
        self.marker_pub=self.create_publisher(MarkerArray,PREFIX+'/speed_markers',1)
        self.create_subscription(Odometry,'/odometry/local',self.on_pose,20)
        self.create_subscription(TerrainAssessment,'/depth_elevation_mapper/planner_costmap',self.on_map,1)
        self.create_subscription(String,'/localization/initialization/status',self.on_heading,1)
        self.create_service(Trigger,'~/reset_reference',self.on_reset_reference)
        self.add_on_set_parameters_callback(self.validate)
        self.create_timer(1./self.get_parameter('preview_rate_hz').value,self.tick)

    def validate(self, changes):
        """表示modeと予測条件を検証する。rate変更はtimer再生成が必要なので対象外。"""
        bounds=dict(reference_length_m=(1.,30.),target_speed=(.05,1.),footprint_radius_m=(.1,1.))
        for p in changes:
            if p.name=='mode':
                if p.value not in ('obstacle_only','full_terrain'):
                    return SetParametersResult(successful=False,reason='modeが不正')
            elif p.name=='reference_mode':
                if p.value not in ('fixed_start','current_pose'):
                    return SetParametersResult(successful=False,reason='reference_modeが不正')
            elif p.name in ('footprint_margin_m','clearance_distance_m','clearance_cost_weight','clearance_min_speed_mps'):
                if not isinstance(p.value,(int,float)) or not math.isfinite(p.value) or p.value<0:
                    return SetParametersResult(successful=False,reason=p.name+'は非負の有限値です')
            elif p.name in bounds:
                lo,hi=bounds[p.name]
                if not isinstance(p.value,(int,float)) or not math.isfinite(p.value) or not lo<=p.value<=hi:
                    return SetParametersResult(successful=False,reason=p.name+'が範囲外')
                if p.name=='target_speed' and p.value>self.limits.max_speed:
                    return SetParametersResult(successful=False,reason='車体設定の速度上限を超えています')
            else:return SetParametersResult(successful=False,reason=p.name+'は実行中変更に未対応')
        self.solver=None
        if any(p.name in ('reference_length_m','reference_mode') and p.value!=self.get_parameter(p.name).value for p in changes):
            self.reset_reference=True
        return SetParametersResult(successful=True)

    def on_reset_reference(self, request, response):
        self.reset_reference=True;response.success=True
        response.message='次の有効poseを基準に直進経路を作り直します';return response

    def on_pose(self, message):
        """local EKFの実測poseを保持。TFの所有者は既存composerだけとする。"""
        if message.header.frame_id!=FRAME or message.child_frame_id!='base_link':return
        p=message.pose.pose.position;q=message.pose.pose.orientation
        values=(p.x,p.y,p.z,q.x,q.y,q.z,q.w)
        if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:])-1.)>.01:return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.pose=(p.x,p.y,yaw);self.pose_received=self.input_time();self.pose_stamp=message.header.stamp
        self.velocity=(message.twist.twist.linear.x,message.twist.twist.angular.z)

    def on_map(self,message):
        if message.schema_version!=1 or message.header.frame_id!=FRAME:return
        self.assessment=message;self.map_received=self.input_time()

    def input_time(self):
        """bagの低速再生・pauseでは入力の古さもsim時刻で数え、壁時計と混ぜない。"""
        return (self.get_clock().now().nanoseconds/1e9 if self.get_parameter('use_sim_time').value
                else time.monotonic())

    def on_heading(self,message):
        try:self.heading_ready=json.loads(message.data).get('state')=='READY'
        except (ValueError,TypeError):self.heading_ready=False

    def path(self, points):
        """メートル座標の参照線・予測線を同一frameのPathへ変換する。"""
        msg=Path();msg.header.frame_id=FRAME;msg.header.stamp=self.get_clock().now().to_msg()
        for point in points:
            pose=PoseStamped();pose.header=msg.header
            pose.pose.position.x=float(point[0]);pose.pose.position.y=float(point[1]);pose.pose.orientation.w=1.
            msg.poses.append(pose)
        return msg

    def publish_speed(self, result=None, environment=None):
        """採用軌道だけを表示する。失敗時は古い線・ラベル・グラフを消去する。

        capはsolverと同じ各区間の開始位置で計算する。加減速制限のため、
        採用速度がこの上限を一時的に超える場合も、そのまま区別して表示する。
        色の範囲は車体の最大速度に固定し、フレーム間で自動伸縮しない。
        """
        markers=MarkerArray()
        clear=Marker();clear.action=Marker.DELETEALL;markers.markers.append(clear)
        data={'valid':False}
        if result is not None:
            trajectory=np.asarray(result['trajectory'])
            speeds=np.asarray(result['trajectory_controls'])[:,0]
            xy=np.vstack((np.asarray(self.pose[:2]),trajectory[:,:2]))
            distance=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
            c=self.solver.c
            risk=np.asarray(environment.clearance_risk(xy[:-1,0],xy[:-1,1],c.clearance_distance_m))
            caps=np.minimum(c.max_speed,c.target_speed-(c.target_speed-min(c.clearance_min_speed_mps,c.target_speed))*risk)
            data=dict(valid=True,distance_m=distance[:-1].tolist(),speed_mps=speeds.tolist(),
                      cap_mps=np.broadcast_to(caps,speeds.shape).tolist(),target_mps=c.target_speed,
                      actual_mps=float(self.velocity[0]),color_max_mps=float(self.limits.max_speed),
                      dt_sec=c.dt)
            header=self.path([]).header
            line=Marker();line.header=header;line.ns='planned_speed';line.id=0
            line.type=Marker.LINE_STRIP;line.action=Marker.ADD;line.pose.orientation.w=1.
            line.scale.x=.12
            # 最初の区間にも速度色を付け、実測現在位置から予測終点まで連続表示する。
            for point,v in zip(xy,np.r_[speeds[0],speeds]):
                line.points.append(Point(x=float(point[0]),y=float(point[1]),z=.12))
                ratio=float(np.clip(v/self.limits.max_speed,0.,1.))
                line.colors.append(ColorRGBA(r=min(1.,2*(1-ratio)),g=min(1.,2*ratio),b=0.,a=1.))
            markers.markers.append(line)
            stride=max(1,int(round(.5/c.dt)))
            indices=sorted(set([0,len(speeds)-1]+list(range(stride-1,len(speeds),stride))))
            for k in indices:
                label=Marker();label.header=header;label.ns='speed_labels';label.id=k+1
                label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD;label.pose.orientation.w=1.
                label.pose.position=Point(x=float(trajectory[k,0]),y=float(trajectory[k,1]),z=.35)
                label.scale.z=.14;label.color=ColorRGBA(r=1.,g=1.,b=1.,a=1.)
                label.text='%.1fs: %.2f m/s'%((k+1)*c.dt,speeds[k]);markers.markers.append(label)
        self.marker_pub.publish(markers)
        self.speed_pub.publish(String(data=json.dumps(data,allow_nan=False)))

    def tick(self):
        """unknown・入力停止なら予測を消す。過去の成功pathを残して誤認させない。"""
        status='VIO初期化・実測pose待ち'
        try:
            now=self.input_time();timeout=self.get_parameter('input_timeout_sec').value
            if self.pose is None or now-self.pose_received>timeout:raise ValueError(status)
            stamp=self.pose_stamp.sec+self.pose_stamp.nanosec*1e-9
            if not -.1<=self.get_clock().now().nanoseconds/1e9-stamp<=timeout:
                raise ValueError('local odometryのstampが期限切れ')
            if self.get_parameter('require_heading_ready').value and not self.heading_ready:
                raise ValueError('local EKFの方位初期化待ち。静止させてください')
            if self.reset_reference or self.get_parameter('reference_mode').value=='current_pose':
                x,y,yaw=self.pose;length=self.get_parameter('reference_length_m').value
                distance=np.linspace(0.,length,int(length/.1)+1)
                self.reference=np.column_stack((x+distance*np.cos(yaw),y+distance*np.sin(yaw)))
                self.solver=None;self.reset_reference=False
            if self.solver is None:
                self.solver=MppiPoc(MppiConfig(target_speed=self.get_parameter('target_speed').value,
                    steps=self.get_parameter('prediction_steps').value,
                    dt=self.get_parameter('prediction_dt').value,tracking_mode='corridor',path_tolerance_m=.6,
                    max_path_error=5.,max_projection_advance=2.,min_radius=self.limits.radius,
                    max_speed=self.limits.max_speed,max_yaw_rate=self.limits.max_yaw_rate,
                    max_yaw_accel=self.limits.max_yaw_accel,
                    **{k:self.get_parameter(k).value for k in
                       ('clearance_distance_m','clearance_cost_weight','clearance_min_speed_mps')}))
                self.solver.set_path(self.reference)
                # mode変更時は同じ参照線を維持し、手動移動後の現在位置に対応する弧長から再開。
                delta=np.linalg.norm(self.reference-np.asarray(self.pose[:2]),axis=1)
                self.solver.station=float(self.solver.s[int(np.argmin(delta))])
            self.reference_pub.publish(self.path(self.reference))
            if self.assessment is None or now-self.map_received>timeout:raise ValueError('認識map待ち・受信停止')
            msg=self.assessment;info=msg.info
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if not -.1<=self.get_clock().now().nanoseconds/1e9-stamp<=timeout:
                raise ValueError('認識mapの撮像時刻が期限切れ')
            mode=self.get_parameter('mode').value
            values=select_costs(msg.obstacle_cost,msg.terrain_cost,info.width*info.height,mode)
            grid=OccupancyGrid();grid.header=msg.header;grid.info=info;grid.data=values.astype(np.int8).tolist()
            self.map_pub.publish(grid)
            q=info.origin.orientation
            if abs(q.x)+abs(q.y)+abs(q.z)>1e-6 or abs(q.w-1.)>1e-6:
                raise ValueError('previewはyaw=0のgrid原点だけに対応')
            # 同じ受信mapの再表示では膨張・距離変換を再実行しない。
            key=(id(msg),mode,self.get_parameter('footprint_radius_m').value,
                 self.get_parameter('footprint_margin_m').value)
            if key!=self.cached_environment_key:
                self.cached_environment=TerrainCost(values,info.width,info.height,info.resolution,
                    (info.origin.position.x,info.origin.position.y,0.),key[2],margin_m=key[3])
                self.cached_environment_key=key
                self.cached_environment_message=msg  # id再利用による別mapとの誤一致を防ぐ
            environment=self.cached_environment
            # Joy操縦中の実測速度から予測する。算出したcommandはpublishしない。
            result=self.solver.step(self.pose,self.velocity,previous_command=self.velocity,environment=environment)
            self.path_pub.publish(self.path(result['trajectory']))
            self.publish_speed(result,environment)
            # 時間horizon×目標速度を、実際に最適化された距離と混同しないよう表示する。
            xy=np.vstack((np.asarray(self.pose[:2]),result['trajectory'][:,:2]))
            length=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum())
            horizon=self.solver.c.steps*self.solver.c.dt
            status='%s / 予測窓%.2f秒・軌道長%.2fm（表示のみ・速度指令なし）'%(mode,horizon,length)
        except ValueError as exc:
            self.path_pub.publish(self.path([]));self.publish_speed();status=str(exc)
        self.status_pub.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args);node=PerceptionPreview()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown() if hasattr(rclpy,'try_shutdown') else rclpy.shutdown()
