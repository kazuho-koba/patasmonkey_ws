"""受領mission・MPPI・車両許可を接続する実行管理。起動時は必ず無効。"""
from dataclasses import asdict
import hashlib
import json
import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry,Path,OccupancyGrid
from tf2_ros import Buffer,TransformListener
from .terrain_cost import TerrainCost
from .perception_input import MODES,select_costs
from pm_msgs.msg import TerrainAssessment
from pm_msgs.srv import SetPlannerMode
from std_msgs.msg import String,Bool
from std_srvs.srv import SetBool,Trigger
from ament_index_python.packages import get_package_share_directory
from pm_mission.model import validate
from .io import read_limits,read_vehicle_parameters
from .mppi_poc import MppiConfig,NoValidTrajectoryError
from .follower import RouteFollower


class ExecutionManager(Node):
    """GUIの明示許可と新しい入力が揃う間だけ自律指令を出す。

    手動介入・停止・期限切れはラッチし、復旧しただけでは再開しない。
    SetBoolの非同期応答と実車statusの両方を確認してから出力を許可する。
    """
    def __init__(self,parameter_overrides=None):
        # FoxyのTransformListenerは相対名tf/tf_staticで購読する。
        # plannerのnamespaceへ閉じ込めず、車両全体のTFを購読する。
        super().__init__('execution_manager',namespace='/pm/planner',parameter_overrides=parameter_overrides,
            cli_args=['--ros-args','-r','tf:=/tf','-r','tf_static:=/tf_static'])
        defaults=dict(vehicle_config=get_package_share_directory('pm_config')+'/config/vehicle_control.yaml',
            pose_topic='/odometry/global',base_frame='base_link',controller_rate_hz=10.,
            pose_timeout_sec=.5,operator_timeout_sec=1.5,vehicle_timeout_sec=1.5,preview_timeout_sec=1.5,
            command_timeout_sec=.3,permission_timeout_sec=2.,allow_unverified_manual_anchor=False,
            pass_radius_m=.5,stop_radius_m=.25,start_radius_m=.5,max_deviation_m=1.,max_advance_m=2.,
            jump_slack_m=.3,max_pose_gap_sec=1.,stop_speed_mps=.03,heading_tolerance_rad=.52,resume_ambiguity_distance_m=.15,resume_ambiguity_station_m=1.,
            pose_max_position_std_m=1.,pose_max_yaw_std_rad=.5,pose_jump_slack_m=.3,
            pose_yaw_jump_slack_rad=.2,resume_max_yaw_rate=.05,
            keep_autonomy_on_blocked=True,perception_mode='',perception_costmap_topic='/depth_elevation_mapper/planner_costmap',
            allow_legacy_full_hazard=False,perception_enabled=False,terrain_topic='/depth_elevation_mapper/terrain_hazard_debug',
            terrain_timeout_sec=1.,footprint_radius_m=.4,footprint_margin_m=.1,terrain_lethal_threshold=100)
        defaults.update({k:v for k,v in asdict(MppiConfig()).items() if k not in
                         ('min_radius','max_speed','max_yaw_rate','max_yaw_accel','dt','yaw_response_gain')})
        defaults['target_speed']=1.
        for k,v in defaults.items():self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in defaults}
        for k in ('pose_timeout_sec','operator_timeout_sec','vehicle_timeout_sec','preview_timeout_sec',
                  'command_timeout_sec','permission_timeout_sec','controller_rate_hz',
                  'resume_ambiguity_distance_m','resume_ambiguity_station_m','pose_max_position_std_m',
                  'pose_max_yaw_std_rad','pose_jump_slack_m','pose_yaw_jump_slack_rad','resume_max_yaw_rate',
                  'terrain_timeout_sec','footprint_radius_m'):
            if not math.isfinite(self.p[k]) or self.p[k]<=0:raise ValueError(k+'は正の有限値が必要です')
        if not math.isfinite(self.p['footprint_margin_m']) or self.p['footprint_margin_m']<0:raise ValueError('footprint_margin_mは非負の有限値[m]が必要です')
        if not 1<=self.p['controller_rate_hz']<=20:raise ValueError('controller_rate_hzは1〜20です')
        self.limits=read_limits(self.p['vehicle_config']);self.follower=RouteFollower(self.limits,self.p)
        if read_vehicle_parameters(self.p['vehicle_config']).get('cmd_vel_topic','/cmd_vel_autonomy')!='/cmd_vel_autonomy':
            raise ValueError('車両の自律入力topicを/cmd_vel_autonomyに合わせてください')
        self.perception_mode=self.p['perception_mode'] or ('full_terrain' if self.p['perception_enabled'] else 'off')
        if self.perception_mode not in MODES:raise ValueError('perception_modeが不正です')
        self.p['perception_enabled']=self.perception_mode!='off'
        self.last_assessment_stamp=None;self.mode_stamp_ns=0;self.assessment_drop_count=0
        self.mission=None;self.mission_document=None;self.preview=None;self.preview_received=None
        self.vehicle=None;self.vehicle_received=None;self.pose=None;self.pose_received=None;self.pose_error='自己位置未受信'
        self.operator_received=None;self.operator_session=None;self.running_session=None
        self.state='DISABLED';self.reason='ミッション未受領';self.last_rejection=''
        self.permission_future=None;self.stop_future=None;self.desired_permission=False;self.permission_ack=False
        self.waiting_since=None
        self.last_command_time=None;self.last_compute_ms=None;self.last_permission_time=0.;self.last_status_time=0.
        self.pending_reference=None
        self.terrain=None;self.terrain_received=None;self.terrain_stamp=None;self.terrain_frame=None
        self.terrain_error='hazard map未受信'
        self.tf_buffer=Buffer();self.tf_listener=TransformListener(self.tf_buffer,self)
        self.emergency=False
        self.pose_fault='';self.pose_history=None
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TerrainAssessment,self.p['perception_costmap_topic'],self.on_assessment,1)
        if self.p['allow_legacy_full_hazard']:
            self.create_subscription(OccupancyGrid,self.p['terrain_topic'],self.on_terrain,1)
        self.create_subscription(String,'/pm/mission/accepted',self.on_mission,latched)
        self.create_subscription(String,'/pm/planner/preview/reference',self.on_reference,latched)
        self.create_subscription(String,'/pm/planner/preview/status',self.on_preview,10)
        self.create_subscription(String,'/pm/vehicle/status',self.on_vehicle,10)
        self.create_subscription(Odometry,self.p['pose_topic'],self.on_pose,qos_profile_sensor_data)
        self.create_subscription(String,'/pm/planner/operator_heartbeat',self.on_operator,1)
        self.create_subscription(Twist,'/cmd_vel_joy',self.on_joy,10)
        self.create_subscription(Bool,'/emergency_stop',self.on_emergency,10)
        self.mission_publisher=self.create_publisher(String,'~/mission',latched)
        self.publisher=self.create_publisher(String,'~/status',10)
        self.command_publisher=self.create_publisher(Twist,'/cmd_vel_autonomy',1)
        self.local_path_publisher=self.create_publisher(Path,'~/local_path',1)
        self.trajectory_publisher=self.create_publisher(String,'~/trajectory',1)
        self.vehicle_client=self.create_client(SetBool,'/pm/vehicle/set_autonomy_enabled')
        self.create_service(SetPlannerMode,'~/set_perception_mode',self.set_perception_mode)
        self.create_service(Trigger,'~/start',self.start)
        self.create_service(Trigger,'~/stop',self.stop)
        self.control_timer=self.create_timer(1./self.p['controller_rate_hz'],self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_mission(self,message):
        """新しい受領・再起動通知では旧速度を無効化し、hashを再検証する。"""
        self.halt('新しいミッションを受領しました')
        self.pose=None;self.pose_received=None
        self.pose_fault='';self.pose_history=None
        self.follower=RouteFollower(self.limits,self.p);self.preview=None;self.preview_received=None
        try:
            payload=json.loads(message.data);document=validate(payload['mission'],for_submission=True)
            digest=hashlib.sha256(json.dumps(document,sort_keys=True).encode()).hexdigest()
            if digest!=payload['sha256']:raise ValueError('mission hashが一致しません')
            self.mission=dict(mission_id=document['mission_id'],revision=document['revision'],name=document['name'],
                              sha256=digest,waypoint_count=len(document['waypoints']),stored_path=payload['stored_path'])
            self.mission_document=document
            self.mission_publisher.publish(String(data=json.dumps(dict(mission=document,sha256=digest),ensure_ascii=False)))
            self.reason='経路受領済み。参照Pathを待機中'
            if self.pending_reference:self.accept_reference(self.pending_reference)
        except (ValueError,TypeError,KeyError) as exc:
            self.mission=None;self.mission_document=None;self.mission_publisher.publish(String(data=''))
            self.follower=RouteFollower(self.limits,self.p)
            self.reason='経路の検証失敗: '+str(exc)

    def accept_reference(self,data):
        if not self.mission or data.get('mission_sha256')!=self.mission['sha256'] or \
                data.get('mission_id')!=self.mission['mission_id'] or data.get('revision')!=self.mission['revision']:
            return
        if [e['waypoint_id'] for e in data['events']]!=[w['waypoint_id'] for w in self.mission_document['waypoints']]:
            raise ValueError('参照Pathのwaypoint IDが受領missionと不一致')
        if [e['attribute'] for e in data['events']]!=[w['attribute'] for w in self.mission_document['waypoints']]:
            raise ValueError('参照Pathの必須到達属性が受領missionと不一致')
        provenance=data.get('coordinate_provenance',{})
        if provenance.get('mode')=='manual' and not self.p['allow_unverified_manual_anchor']:
            if provenance.get('anchor',{}).get('alignment_verified') is not True:raise ValueError('manual anchorの座標整合が未確認です')
        self.follower.load(data);self.reason='参照Path受信。明示開始を待機中'

    def on_reference(self,message):
        self.halt('参照Path更新により無効化しました')
        self.pose=None;self.pose_received=None
        self.follower=RouteFollower(self.limits,self.p)
        try:
            self.pending_reference=json.loads(message.data) if message.data else None
            if self.pending_reference:self.accept_reference(self.pending_reference)
        except (ValueError,KeyError,TypeError,IndexError,AttributeError) as exc:
            self.pending_reference=None;self.follower=RouteFollower(self.limits,self.p)
            self.reason='MPPI/Path準備エラー: '+str(exc)

    def on_preview(self,message):
        try:
            data=json.loads(message.data);provenance=data.get('coordinate_provenance') or {}
            if not self.mission or data.get('mission_id')!=self.mission['mission_id'] or \
                    data.get('revision')!=self.mission['revision'] or provenance.get('mission_sha256')!=self.mission['sha256']:return
            self.preview=data;self.preview_received=time.monotonic()
        except (ValueError,TypeError,AttributeError):
            self.preview=None;self.preview_received=None

    def on_vehicle(self,message):
        try:
            data=json.loads(message.data)
            if not isinstance(data.get('autonomy_enabled'),bool):raise ValueError('車両自律許可が不正')
            self.vehicle=data;self.vehicle_received=time.monotonic()
            if self.state in ('RUNNING','WAITING') and not data['autonomy_enabled']:self.halt('車両側で自律許可が解除されました')
        except (ValueError,TypeError,AttributeError):
            self.vehicle=None;self.vehicle_received=None

    def set_perception_mode(self,request,response):
        """階層を一つの列挙値で受け、切替と走行開始を分離する。"""
        mode=request.mode
        response.mode=self.perception_mode
        if mode not in MODES:
            response.success=False;response.message='modeはoff / obstacle_only / full_terrainです';return response
        if mode!=self.perception_mode:
            # ON/OFFどちらの切替でも旧計算結果を送らず、明示開始まで停止する。
            self.halt('認識mode変更。明示的な走行開始が必要です')
            self.perception_mode=mode;self.p['perception_enabled']=mode!='off'
            self.terrain=None;self.terrain_received=None;self.terrain_stamp=None;self.terrain_frame=None
            self.terrain_error='選択modeの新しい専用costmapを待機中'
            self.last_assessment_stamp=None;self.mode_stamp_ns=self.get_clock().now().nanoseconds
        response.success=True;response.mode=self.perception_mode
        response.message='認識modeを'+self.perception_mode+'に設定しました。走行開始は別操作です'
        self.publish_status();return response

    def on_assessment(self,message):
        """同時刻の独立channelを選択。未知の片側を総合hazardの0で隠さない。"""
        if self.perception_mode=='off':return
        try:
            if message.schema_version!=1:raise ValueError('未対応のTerrainAssessment schemaです')
            stamp=message.header.stamp.sec*1000000000+message.header.stamp.nanosec
            if stamp<self.mode_stamp_ns:raise ValueError('mode変更より古いcostmapです')
            # 重複・逆順配送で鮮度を延長したり、旧mapへ巻き戻したりしない。
            if self.last_assessment_stamp is not None and stamp<=self.last_assessment_stamp:
                self.assessment_drop_count+=1;return
            size=message.info.width*message.info.height
            costs=select_costs(message.obstacle_cost,message.terrain_cost,size,self.perception_mode,
                               self.p['terrain_lethal_threshold'])
            grid=OccupancyGrid();grid.header=message.header;grid.info=message.info
            grid.data=costs.astype('int8').tolist()
            self.install_terrain(grid)
            if self.terrain is not None:self.last_assessment_stamp=stamp
        except Exception as exc:
            self.terrain=None;self.terrain_error='専用costmap使用不可: '+str(exc)
        if self.terrain is None and self.state in ('STARTING','RUNNING','WAITING'):self.halt(self.terrain_error)

    def on_terrain(self,message):
        """旧debug総合mapの明示opt-in用。障害物未知を区別できないため既定では使わない。"""
        if not self.p['allow_legacy_full_hazard'] or self.perception_mode!='full_terrain':return
        if message.header.stamp.sec*1000000000+message.header.stamp.nanosec<self.mode_stamp_ns:return
        self.install_terrain(message)

    @staticmethod
    def planar_yaw(q):
        """平面costmapの姿勢を検証する。傾斜したgridをXYへ暗黙に潰さない。"""
        if not all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w)) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.02:
            raise ValueError('hazard/TFの姿勢が不正')
        if abs(q.x)>.025 or abs(q.y)>.025:raise ValueError('hazard/TFの傾きが大きすぎます')
        return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

    def install_terrain(self,message):
        """hazard受信時だけfootprint膨張。grid撮像stampのTFでPath frameへ揃える。"""
        try:
            if not self.follower.snapshot:raise ValueError('Path未受信')
            frame=self.follower.snapshot['frame_id'];origin=message.info.origin
            yaw=self.planar_yaw(origin.orientation);x=origin.position.x;y=origin.position.y
            stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
            age=self.get_clock().now().nanoseconds/1e9-stamp
            if not -.1<=age<=self.p['terrain_timeout_sec']:raise ValueError('hazard mapのstampが期限切れ/未来')
            if not message.header.frame_id:raise ValueError('hazard mapのframeが空です')
            if message.header.frame_id!=frame:
                # latest TFでの代用はしない。TF未到達なら次のmapで再試行する。
                tf=self.tf_buffer.lookup_transform(frame,message.header.frame_id,rclpy.time.Time.from_msg(message.header.stamp)).transform
                angle=self.planar_yaw(tf.rotation);c=math.cos(angle);s=math.sin(angle)
                x,y=c*x-s*y+tf.translation.x,s*x+c*y+tf.translation.y;yaw+=angle
            terrain=TerrainCost(message.data,message.info.width,message.info.height,message.info.resolution,
                                (x,y,yaw),self.p['footprint_radius_m'],self.p['terrain_lethal_threshold'],margin_m=self.p['footprint_margin_m'])
            self.terrain=terrain;self.terrain_stamp=stamp;self.terrain_received=time.monotonic()
            self.terrain_frame=frame;self.terrain_error=''
        except Exception as exc:
            # 変換失敗した新mapの代わりに古いmapで走り続けない。
            self.terrain=None;self.terrain_error='hazard map使用不可: '+str(exc)

    def on_operator(self,message):
        if not message.data:return
        if self.state in ('STARTING','RUNNING','WAITING') and message.data!=self.running_session:
            self.halt('操作GUIのセッションが変わりました')
        self.operator_session=message.data;self.operator_received=time.monotonic()

    def on_joy(self,message):
        if self.state in ('STARTING','RUNNING','WAITING'):self.halt('手動介入。GUIから明示的な再開が必要です')

    def on_emergency(self,message):
        self.emergency=bool(message.data)
        if message.data:self.halt('緊急停止')

    def on_pose(self,message):
        """Pathと同一frameの新しいposeを受ける。暗黙のlatest TFは使わない。"""
        try:
            frame=self.follower.snapshot['frame_id'] if self.follower.snapshot else None
            if frame is None:raise ValueError('Path未受信')
            if message.header.frame_id!=frame or message.child_frame_id!=self.p['base_frame']:
                raise ValueError('Path/Odometry frame不一致')
            q=message.pose.pose.orientation
            if not all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w)) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.02:
                raise ValueError('姿勢quaternionが不正')
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            t=message.header.stamp.sec+message.header.stamp.nanosec/1e9;p=message.pose.pose.position
            v=message.twist.twist.linear.x;w=message.twist.twist.angular.z
            pose=(t,p.x,p.y,yaw,v,w)
            # x/y/yawの共分散[ m² / rad² ]から開始・継続を拒否する。
            # 全ゼロはROS上で品質不明のため、正常の証明とは扱わない（別検査も必要）。
            cov=message.pose.covariance
            if not all(math.isfinite(c) for c in cov) or any(cov[i]<0 for i in (0,7,35)):
                raise ValueError('自己位置共分散が不正')
            xy_max=.5*(cov[0]+cov[7]+math.sqrt((cov[0]-cov[7])**2+(cov[1]+cov[6])**2))
            if xy_max>self.p['pose_max_position_std_m']**2 or cov[35]>self.p['pose_max_yaw_std_rad']**2:
                raise ValueError('自己位置の推定不確かさが上限超過')
            if not all(math.isfinite(x) for x in pose):raise ValueError('自己位置/速度が非有限')
            age=self.get_clock().now().nanoseconds/1e9-t
            if not -.1<=age<=self.p['pose_timeout_sec']:raise ValueError('自己位置stampが期限切れ/未来')
            # 手動中も直前の有効poseと比較し、停止中に生じた破綻を見逃さない。
            old=self.pose_history
            if old is not None:
                dt=t-old[0]
                if dt<0:self.pose_fault='自己位置の時刻巻戻り'
                elif dt>0:
                    shift=math.hypot(p.x-old[1],p.y-old[2])
                    turn=abs(math.atan2(math.sin(yaw-old[3]),math.cos(yaw-old[3])))
                    if shift>self.limits.max_speed*dt+self.p['pose_jump_slack_m'] or turn>self.limits.max_yaw_rate*dt+self.p['pose_yaw_jump_slack_rad']:
                        self.pose_fault='自己位置の不自然なジャンプ。推定を確認してmissionを再受領してください'
                elif any(abs(a-b)>1e-8 for a,b in zip(pose[1:4],old[1:4])):
                    self.pose_fault='同一stampで自己位置が変化しました'
            if self.pose_fault:raise ValueError(self.pose_fault)
            self.pose_history=pose
            self.pose=pose;self.pose_received=time.monotonic();self.pose_error=''
        except (ValueError,TypeError) as exc:
            self.pose=None;self.pose_received=None;self.pose_error=str(exc)
            if self.state in ('STARTING','RUNNING','WAITING'):self.halt(self.pose_error)

    def fresh(self,stamp,timeout):return stamp is not None and time.monotonic()-stamp<timeout

    def readiness(self,check_footprint=True):
        """開始・継続の両方に使う条件。停止中の車両許可OFFは準備不足ではない。"""
        if not self.mission:return 'ミッション未受領'
        if self.pose_fault:return self.pose_fault
        if self.emergency:return '緊急停止'
        if not self.follower.snapshot:return 'MPPI/Path準備待ち'
        if self.p['perception_enabled']:
            if self.terrain is None:return self.terrain_error
            if self.terrain_frame!=self.follower.snapshot['frame_id']:return 'hazard/Path frame不一致'
            if not self.fresh(self.terrain_received,self.p['terrain_timeout_sec']):return 'hazard map受信が期限切れ'
            age=self.get_clock().now().nanoseconds/1e9-self.terrain_stamp
            if not -.1<=age<=self.p['terrain_timeout_sec']:return 'hazard mapのstampが期限切れ/未来'
        if not self.fresh(self.preview_received,self.p['preview_timeout_sec']):return 'preview状態が期限切れ'
        if not self.preview.get('geometry_valid') or self.preview.get('load_state')!='READY':return '参照Pathが無効'
        if self.preview.get('path_id')!=self.follower.snapshot['path_id']:return '参照Pathの版が不一致'
        if not self.fresh(self.operator_received,self.p['operator_timeout_sec']):return '操作GUI heartbeatが期限切れ'
        if not self.fresh(self.vehicle_received,self.p['vehicle_timeout_sec']):return '車両状態が期限切れ'
        if not self.vehicle.get('connected') or not self.vehicle.get('armed') or self.vehicle.get('emergency_stop',True):
            return '車両未接続・走行未許可または緊急停止'
        if not self.fresh(self.pose_received,self.p['pose_timeout_sec']) or self.pose is None:return self.pose_error or '自己位置が期限切れ'
        age=self.get_clock().now().nanoseconds/1e9-self.pose[0]
        if not -.1<=age<=self.p['pose_timeout_sec']:return '自己位置stampが期限切れ/未来'
        if check_footprint:
            blocked=self.footprint_block_reason()
            if blocked:return blocked
        return ''

    def footprint_block_reason(self):
        """入力の健全性を検査した後に、現在の車体範囲だけの通行可否を調べる。"""
        if self.p['perception_enabled'] and self.terrain.sample(self.pose[1],self.pose[2])>=self.p['terrain_lethal_threshold']:
            return '現在のfootprintが危険または評価不能です'
        return ''

    def wait_on_blocked(self,reason,legacy_state='FAULT'):
        """健全な入力での通行不可だけを待機扱いにし、車両許可は維持する。"""
        if not self.p['keep_autonomy_on_blocked']:
            self.halt(reason,legacy_state);return
        self.follower.hold(self.pose[:5])
        entering=self.state!='WAITING'
        self.state='WAITING';self.reason='障害物待機（自律許可維持）: '+reason
        if entering:
            self.waiting_since=time.monotonic();self.clear_local_path()
        # tick末尾でゼロを送信する。待機中も指令期限を監視し、executor停止は別途無効化する。
        self.last_command_time=time.monotonic()

    def start(self,request,response):
        reason=self.readiness()
        if self.state in ('STARTING','RUNNING','WAITING'):reason='自律走行は開始済みです'
        if self.permission_future and not self.permission_future.done():reason='車両許可の応答待ちです'
        if not self.vehicle_client.service_is_ready():reason=reason or '車両許可サービス未接続'
        if not reason and abs(self.pose[5])>self.p['resume_max_yaw_rate']:reason='開始・再開には旋回停止の確認が必要です'
        if not reason:
            try:self.follower.start(self.pose[:5])
            except ValueError as exc:reason=str(exc)
        if reason:
            response.success=False;response.message=reason;self.last_rejection=reason;self.reason=reason;return response
        self.running_session=self.operator_session;self.state='STARTING';self.reason='車両自律許可を確認中'
        self.desired_permission=True;self.permission_ack=False;self.request_permission()
        response.success=True;response.message='開始要求を受け付けました。実際の有効状態はstatusで確認してください'
        return response

    def stop(self,request,response):
        self.halt('GUIからの終了要求')
        self.desired_permission=False;self.request_permission(force=True)
        response.success=True;response.message='planner速度出力を無効化しました。車両側の許可解除を要求します'
        return response

    def halt(self,reason,state='DISABLED'):
        was_active=self.state in ('STARTING','RUNNING','WAITING')
        self.state=state;self.reason=reason;self.desired_permission=False;self.permission_ack=False
        self.last_command_time=None;self.waiting_since=None;self.follower.pause(reason)
        if hasattr(self,'command_publisher'):
            self.command_publisher.publish(Twist());self.clear_local_path()
        if was_active:self.request_permission()

    def request_permission(self,force=False):
        """true/false要求を直列化し、遅いtrue応答が停止後の再許可にならないようにする。"""
        if self.permission_future and not self.permission_future.done():return
        if not self.vehicle_client.service_is_ready():return
        if not force and not self.desired_permission and not (self.vehicle and self.vehicle.get('autonomy_enabled')):return
        req=SetBool.Request();req.data=self.desired_permission
        self.last_permission_time=time.monotonic();requested=req.data
        future=self.vehicle_client.call_async(req);self.permission_future=future
        if not requested:self.stop_future=future
        def completed(f):
            try:
                result=f.result()
                if requested and self.desired_permission and self.state=='STARTING':
                    if result.success:self.permission_ack=True
                    else:self.halt('車両許可拒否: '+result.message)
                if requested and not self.desired_permission:self.request_permission(force=True)
            except Exception as exc:
                if requested:self.halt('車両許可応答エラー: '+str(exc))
        future.add_done_callback(completed)

    def clear_local_path(self):
        frame=self.follower.snapshot['frame_id'] if self.follower.snapshot else ''
        msg=Path();msg.header.frame_id=frame;msg.header.stamp=self.get_clock().now().to_msg()
        self.local_path_publisher.publish(msg)
        self.trajectory_publisher.publish(String(data=json.dumps(dict(valid=False,reason=self.reason))))

    def publish_trajectory(self,result):
        msg=Path();msg.header.frame_id=self.follower.snapshot['frame_id'];msg.header.stamp=self.get_clock().now().to_msg()
        controls=result['trajectory_controls'];dt=self.follower.solver.c.dt;points=[]
        for i,((x,y,yaw),(v,w)) in enumerate(zip(result['trajectory'],controls)):
            pose=PoseStamped();pose.header=msg.header;pose.pose.position.x=float(x);pose.pose.position.y=float(y)
            pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2);msg.poses.append(pose)
            points.append(dict(time_from_start_sec=(i+1)*dt,x=float(x),y=float(y),yaw=float(yaw),v=float(v),w=float(w)))
        self.local_path_publisher.publish(msg)
        self.trajectory_publisher.publish(String(data=json.dumps(dict(valid=True,frame_id=msg.header.frame_id,
            stamp_sec=self.get_clock().now().nanoseconds/1e9,path_id=self.follower.snapshot['path_id'],points=points),allow_nan=False)))

    def tick(self):
        now=time.monotonic();command=Twist()
        if self.state in ('RUNNING','WAITING') and self.last_command_time is not None and now-self.last_command_time>self.p['command_timeout_sec']:
            self.halt('制御周期が期限切れ。明示再開が必要です')
        if self.state in ('STARTING','RUNNING','WAITING'):
            # 入力欠落・自己位置異常・手動停止は待機へ緩和しない。
            reason=self.readiness(check_footprint=False)
            if reason:self.halt(reason)
            elif self.state=='STARTING':
                if self.permission_ack and self.vehicle.get('autonomy_enabled'):
                    self.state='RUNNING';self.reason='経路追従中'
                elif now-self.last_permission_time>self.p['permission_timeout_sec']:
                    self.halt('車両許可確認が期限切れ')
            if self.state in ('RUNNING','WAITING'):
                try:
                    blocked=self.footprint_block_reason()
                    if blocked:
                        self.wait_on_blocked(blocked,'DISABLED')
                    elif self.state=='WAITING' and (abs(self.pose[4])>self.p['stop_speed_mps'] or abs(self.pose[5])>self.p['resume_max_yaw_rate']):
                        self.wait_on_blocked('停止確認待ち')
                    else:
                        started=time.monotonic();result=self.follower.step(self.pose[:5],self.pose[5],self.terrain if self.p['perception_enabled'] else None)
                        self.last_compute_ms=(time.monotonic()-started)*1000.
                        reason=self.readiness(check_footprint=False)
                        if reason or time.monotonic()-started>self.p['command_timeout_sec']:
                            raise ValueError(reason or 'MPPI計算結果が期限切れ')
                        command.linear.x,command.angular.z=result['command']
                        if not all(math.isfinite(v) for v in (command.linear.x,command.angular.z)):raise ValueError('速度指令が非有限')
                        if not 0.<=command.linear.x<=self.limits.max_speed+1e-8 or abs(command.angular.z)>min(self.limits.max_yaw_rate,command.linear.x/self.limits.radius)+1e-8:
                            raise ValueError('速度指令が車体制約を超えています')
                        if self.follower.progress.state=='COMPLETE':
                            self.halt('全waypoint通過・停止を確認しました','COMPLETED');command=Twist()
                        else:
                            self.state='RUNNING';self.waiting_since=None;self.reason='経路追従中'
                            self.last_command_time=time.monotonic();self.publish_trajectory(result)
                except NoValidTrajectoryError as exc:
                    try:self.wait_on_blocked(str(exc))
                    except (ValueError,ArithmeticError) as error:self.halt('待機中の進捗異常: '+str(error),'FAULT')
                    command=Twist()
                except (ValueError,ArithmeticError,KeyError,TypeError) as exc:
                    self.halt('追従停止: '+str(exc),'FAULT');command=Twist()
        if self.state not in ('STARTING','RUNNING','WAITING'):
            if self.vehicle and self.vehicle.get('autonomy_enabled') and now-self.last_permission_time>1.:
                self.desired_permission=False;self.request_permission()
        self.command_publisher.publish(command)
        if now-self.last_status_time>=.2:self.publish_status();self.last_status_time=now

    def publish_status(self):
        vehicle_fresh=self.fresh(self.vehicle_received,self.p['vehicle_timeout_sec'])
        preview_fresh=self.fresh(self.preview_received,self.p['preview_timeout_sec'])
        preview=self.preview if preview_fresh else None
        reason=self.readiness()
        can_start=not reason and self.state not in ('STARTING','RUNNING','WAITING') and self.follower.progress is not None and self.follower.progress.state in ('READY','MANUAL_PAUSED')
        if self.permission_future and not self.permission_future.done():can_start=False
        input_ready=False
        if self.p['perception_enabled'] and self.terrain is not None and self.pose is not None and self.follower.snapshot:
            age=self.get_clock().now().nanoseconds/1e9-self.terrain_stamp
            input_ready=(self.fresh(self.terrain_received,self.p['terrain_timeout_sec'])
                and -.1<=age<=self.p['terrain_timeout_sec']
                and self.terrain_frame==self.follower.snapshot['frame_id']
                and self.terrain.sample(self.pose[1],self.pose[2])<self.p['terrain_lethal_threshold'])
        status=dict(state=self.state,reason=self.reason,mission=self.mission,last_rejection=self.last_rejection,
            controller_ready=self.follower.solver is not None,drive_ready=bool(can_start),readiness_reason=reason,
            enabled=self.state in ('RUNNING','WAITING'),waiting_for_obstacle=self.state=='WAITING',
            keep_autonomy_on_blocked=self.p['keep_autonomy_on_blocked'],
            waiting_sec=0. if self.waiting_since is None else time.monotonic()-self.waiting_since,vehicle_fresh=vehicle_fresh,
            vehicle_autonomy_enabled=self.vehicle.get('autonomy_enabled') if vehicle_fresh else None,
            path_state=preview.get('load_state','UNKNOWN') if preview else 'WAITING_FOR_PREVIEW' if self.mission else 'NO_MISSION',
            path_error=preview.get('load_error','') if preview else '',path_frame=preview.get('frame_id') if preview else None,
            path_geometry_valid=bool(preview and preview.get('geometry_valid')),path_preview_fresh=preview_fresh,
            operator_fresh=self.fresh(self.operator_received,self.p['operator_timeout_sec']),
            localization_fault=self.pose_fault,
            path_id=self.follower.snapshot['path_id'] if self.follower.snapshot else None,
            progress=self.follower.progress.status() if self.follower.progress else None,
            compute_ms=self.last_compute_ms,perception_enabled=self.p['perception_enabled'],
            perception_mode=self.perception_mode,obstacle_avoidance_enabled=self.perception_mode!='off',
            terrain_evaluation_enabled=self.perception_mode=='full_terrain',
            perception_input_ready=bool(input_ready),perception_input_error=self.terrain_error,
            assessment_drop_count=self.assessment_drop_count,
            environment_checked=bool(self.p['perception_enabled'] and not reason))
        self.publisher.publish(String(data=json.dumps(status,ensure_ascii=False,allow_nan=False)))


def main(args=None):
    rclpy.init(args=args);node=ExecutionManager()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if rclpy.ok():
            node.halt('planner終了');node.desired_permission=False;node.request_permission(force=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
