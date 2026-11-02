"""参照Pathの版を検証し、進捗・MPPI・予測トラジェクトリを保持する。"""
from dataclasses import asdict
import hashlib
import json
import math
import numpy as np
from .reference import Reference,Limits
from .progress import Progress
from .mppi_poc import MppiConfig,MppiPoc


def reference_snapshot(reference,mission,provenance,frame):
    """一つのmessageにPathとmission識別子をまとめ、旧経路との混在を防ぐ。"""
    data=dict(mission_id=mission['mission_id'],revision=mission['revision'],
              mission_sha256=provenance['mission_sha256'],frame_id=frame,
              coordinate_provenance=provenance,samples=reference.samples,events=reference.events,
              limits=asdict(reference.limits),spacing=reference.spacing,turns=reference.turns)
    data['path_id']=hashlib.sha256(json.dumps(data,sort_keys=True,allow_nan=False).encode()).hexdigest()
    return data


class RouteFollower:
    """ROSやモータを所有しない追従器。開始・無効化の許可判断は実行管理が行う。"""
    def __init__(self,limits,parameters):
        self.limits=limits;self.p=parameters;self.snapshot=None;self.reference=None
        self.progress=None;self.solver=None;self.previous=(0.,0.);self.goal_wait=False

    def load(self,data):
        """検証済みpreviewから同一版のPathを受ける。停止属性は未対応として拒否する。"""
        digest=data['path_id'];body={k:v for k,v in data.items() if k!='path_id'}
        if hashlib.sha256(json.dumps(body,sort_keys=True,allow_nan=False).encode()).hexdigest()!=digest:
            raise ValueError('Path hash不一致')
        samples=np.asarray(data['samples'],dtype=float)
        if samples.ndim!=2 or samples.shape[1]!=4 or not 2<=len(samples)<=100000 or not np.isfinite(samples).all():
            raise ValueError('Path点列が不正です')
        if not data['frame_id'] or np.any(np.diff(samples[:,3])<=0):raise ValueError('Path frame/弧長が不正です')
        events=data['events']
        if not isinstance(events,list) or not 2<=len(events)<=len(samples):raise ValueError('Pathイベント数が不正です')
        stations=[]
        for e in events:
            if not all(math.isfinite(float(e[k])) for k in ('x','y','station')):raise ValueError('Pathイベントが非有限です')
            stations.append(float(e['station']))
        if any(b<a for a,b in zip(stations,stations[1:])) or stations[0]<0 or stations[-1]>samples[-1,3]+1e-6:
            raise ValueError('Pathイベントの弧長が不正です')
        if any(e['attribute'] not in ('start','pass','visit','goal') for e in data['events']):
            raise ValueError('現在の自律追従はSTART/PASS/VISIT/GOALのみ対応します')
        if data['events'][0]['attribute']!='start' or data['events'][-1]['attribute']!='goal':
            raise ValueError('開始/終了イベントがありません')
        limits=Limits(**data['limits'])
        if limits.radius<self.limits.radius-1e-8 or abs(limits.max_speed-self.limits.max_speed)>1e-8:
            raise ValueError('previewとcontrollerの車両設定が不一致です。再生成してください')
        if abs(limits.max_yaw_rate-self.limits.max_yaw_rate)>1e-8 or abs(limits.max_yaw_accel-self.limits.max_yaw_accel)>1e-8:
            raise ValueError('previewとcontrollerの旋回制約が不一致です')
        self.reference=Reference(data['samples'],data['events'],limits,data['spacing'],data.get('turns',[]))
        self.snapshot=data
        settings={k:self.p[k] for k in asdict(MppiConfig()) if k in self.p}
        settings.update(min_radius=self.limits.radius,max_speed=self.limits.max_speed,
                        max_yaw_rate=self.limits.max_yaw_rate,max_yaw_accel=self.limits.max_yaw_accel,
                        dt=1./self.p['controller_rate_hz'],yaw_response_gain=1.,
                        max_path_error=self.p['max_deviation_m'])
        self.solver=MppiPoc(MppiConfig(**settings))
        self.solver.set_path(samples[:,:2])
        self.progress=Progress(self.reference,**{key:self.p[name] for key,name in (
            ('pass_radius','pass_radius_m'),('stop_radius','stop_radius_m'),('start_radius','start_radius_m'),
            ('max_deviation','max_deviation_m'),('max_advance','max_advance_m'),('jump_slack','jump_slack_m'),
            ('max_pose_gap','max_pose_gap_sec'),('stop_speed','stop_speed_mps'),('heading_tolerance','heading_tolerance_rad'))})
        self.previous=(0.,0.);self.goal_wait=False

    def start(self,pose):
        if self.progress is None:raise ValueError('Path未受信')
        if self.progress.state=='MANUAL_PAUSED':self.progress.rejoin(pose,self.p['resume_ambiguity_distance_m'],self.p['resume_ambiguity_station_m'])
        else:self.progress.start(pose)
        # 手動後や停止後の旧最適解は再使用しない。進捗弧長は保持する。
        self.previous=(0.,0.);self.solver.nominal.fill(0.)
        self.solver.station=self.progress.station;self.goal_wait=False

    def pause(self,reason):
        if self.progress:self.progress.pause(reason)
        self.previous=(0.,0.)
        if self.solver:self.solver.nominal.fill(0.)

    def step(self,pose,yaw_rate,environment=None):
        self.progress.update(pose)
        if self.progress.state=='FAULT':raise ValueError(self.progress.reason)
        if self.progress.state=='COMPLETE':
            return dict(command=(0.,0.),trajectory=np.array([[pose[1],pose[2],pose[3]]]),
                        trajectory_controls=np.zeros((1,2)),goal=True)
        if self.goal_wait:
            return dict(command=(0.,0.),trajectory=np.array([[pose[1],pose[2],pose[3]]]),
                        trajectory_controls=np.zeros((1,2)),goal=True)
        # 進捗判定と最適化が別の弧長区間を追わないよう、採用済み進捗を下限にする。
        self.solver.station=max(self.solver.station,self.progress.station)
        required=next((e for e in self.reference.events[self.progress.index:] if e['attribute']=='visit'),None)
        self.solver.required_station=required['station'] if required else None
        result=self.solver.step(pose[1:4],(pose[4],yaw_rate),previous_command=self.previous,environment=environment)
        self.previous=result['command']
        if result['goal']:
            if self.progress.index!=len(self.reference.events)-1:raise ValueError('未通過waypointがあるため終点停止します')
            self.goal_wait=True
        return result
