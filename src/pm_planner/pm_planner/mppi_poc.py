"""非駆動MPPI PoC。NumPyで候補を一括計算し、任意の環境cost入力を持ち、車両指令publish自体は行わない。

座標は同一のmetric frame、速度はm/s・rad/s。各制御stepで前進・速度・加速度・
最小半径を制約し、経路の前方弧長だけに沿って候補を評価する。実車用安全機能ではない。
"""
from dataclasses import dataclass
import math
import numpy as np
from .speed_profile import build_speed_profile


class NoValidTrajectoryError(ValueError):
    """新鮮な環境入力に対して衝突しない候補がない。入力異常とは区別する。"""


@dataclass(frozen=True)
class MppiConfig:
    """負荷と予測範囲を調整するPoC設定。乱数seedは再現可能性のため明示する。"""
    batch_size: int = 384
    steps: int = 25
    dt: float = 0.1
    iterations: int = 2
    temperature: float = 0.4
    sigma_v: float = 0.22
    sigma_w: float = 0.65
    target_speed: float = 0.15
    max_speed: float = 1.0
    max_yaw_rate: float = 1.0
    max_yaw_accel: float = 1.0
    min_radius: float = 0.3
    lookahead_m: float = 0.05
    max_projection_advance: float = 0.35
    max_path_error: float = 1.0
    goal_radius: float = 0.10
    path_cost_weight: float = 4.0
    heading_cost_weight: float = 0.2
    speed_cost_weight: float = 0.5
    yaw_cost_weight: float = 0.015
    terminal_cost_weight: float = 5.0
    # interfaceが車体指令を実現する前提。応答の不一致は実験で指定できる。
    yaw_response_gain: float = 1.0
    candidate_sample_count: int = 0
    seed: int = 42
    use_reference_speed_profile: bool = True
    # pointは従来比較用。corridorは横ずれの許容帯を設け、前進を弱く誘導する。
    tracking_mode: str = 'point'
    path_tolerance_m: float = 1.0
    corridor_cost_weight: float = 0.25  # 通常の帯外だけ弱く戻す。必須点の重みとは分ける。
    progress_cost_weight: float = 0.5
    terrain_cost_weight: float = 20.0
    # 必須footprint膨張の外側に希望する追加余裕[m]。0なら離隔評価を無効化。
    clearance_distance_m: float = 0.6
    clearance_cost_weight: float = 3.0
    clearance_min_speed_mps: float = 0.2  # 最大不足時の速度上限。旋回・終点はさらに低速可。

    def __post_init__(self):
        if not (2 <= self.batch_size <= 10000 and 2 <= self.steps <= 200 and 1 <= self.iterations <= 10):
            raise ValueError('候補数・step数・反復数が範囲外です')
        if not 0 <= self.candidate_sample_count <= min(self.batch_size, 64):
            raise ValueError('表示候補数は0〜min(候補数,64)です')
        if not math.isfinite(self.yaw_response_gain) or not 0.0 < self.yaw_response_gain <= 2.0:
            raise ValueError('yaw_response_gainは0より大きく2以下の有限値が必要です')
        for name in ('dt','temperature','sigma_v','sigma_w','target_speed','max_speed','max_yaw_rate',
                     'max_yaw_accel','min_radius','lookahead_m','max_projection_advance','max_path_error','goal_radius'):
            if not math.isfinite(getattr(self,name)) or getattr(self,name) <= 0:
                raise ValueError(name+'は正の有限値が必要です')
        if self.target_speed > self.max_speed:raise ValueError('target_speedが上限を超えています')
        if self.tracking_mode not in ('point','corridor'):raise ValueError('tracking_modeはpoint/corridorです')
        if not math.isfinite(self.path_tolerance_m) or self.path_tolerance_m<0:raise ValueError('path_tolerance_mが不正')
        for name in ('corridor_cost_weight','progress_cost_weight','terrain_cost_weight','path_cost_weight','heading_cost_weight','speed_cost_weight','yaw_cost_weight','terminal_cost_weight','clearance_distance_m','clearance_cost_weight','clearance_min_speed_mps'):
            if not math.isfinite(getattr(self,name)) or getattr(self,name)<0:
                raise ValueError(name+'は非負の有限値が必要です')


class MppiPoc:
    """1経路の進捗と前回最適入力を保持する。別経路には必ずset_pathでresetする。"""
    def __init__(self,config=MppiConfig()):
        self.c=config;self.rng=np.random.default_rng(config.seed)
        self.path=None;self.required_station=None;self.station=0.;self.nominal=np.zeros((config.steps,2),dtype=np.float64)

    def set_path(self,points):
        """Nx2のPathを検証し、弧長s[m]を付加する。重複点と非有限値は拒否する。"""
        p=np.asarray(points,dtype=np.float64)
        if p.ndim!=2 or p.shape[1]!=2 or len(p)<2 or not np.isfinite(p).all():
            raise ValueError('Pathは有限な2次元座標の2点以上が必要です')
        distance=np.linalg.norm(np.diff(p,axis=0),axis=1)
        if np.any(distance<1e-8):raise ValueError('Pathに重複点があります')
        self.path=p.copy();self.s=np.r_[0.,np.cumsum(distance)];self.station=0.
        self.reference_speed,self.reference_curvature=build_speed_profile(
            self.path,self.s,self.c.target_speed,self.c.max_yaw_rate,self.c.max_yaw_accel,
            self.c.max_yaw_accel*self.c.min_radius)
        self.nominal.fill(0.)

    def _project(self,x,y,speed):
        """前進する局所区間だけに投影し、近い往路・復路への飛びを抑える。"""
        # 入力の進行速度に見合う距離＋許容誤差だけ前進を許す。
        # 近接する復路を1周期で数十cm飛ばすことを避ける。
        advance=min(self.c.max_projection_advance,max(.08,abs(speed)*self.c.dt+.05))
        lo=max(0.,self.station-.08);hi=min(self.s[-1],self.station+advance)
        i=max(0,int(np.searchsorted(self.s,lo,side='right'))-1)
        j=min(len(self.path)-1,int(np.searchsorted(self.s,hi,side='left'))+1)
        a=self.path[i:j];b=self.path[i+1:j+1];d=b-a;ds=self.s[i+1:j+1]-self.s[i:j]
        t=np.sum((np.array([x,y])-a)*d,axis=1)/np.sum(d*d,axis=1)
        t=np.clip(t,np.maximum(0.,(lo-self.s[i:j])/ds),np.minimum(1.,(hi-self.s[i:j])/ds))
        projected=a+t[:,None]*d;error=np.sum((projected-np.array([x,y]))**2,axis=1)
        k=int(np.argmin(error));return float(self.s[i+k]+t[k]*ds[k]),math.sqrt(float(error[k]))

    def step(self,pose,velocity,previous_command=None,environment=None):
        """pose=(x,y,yaw), velocity=(v,w)から次指令と採用予測軌道を求める。

        簡易MPPIの重み付き摂動更新を使う。候補は各stepで制約投影し、重みは
        最小costを差し引いたsoftmaxで算出する。environment指定時はhazard・unknown・map外の候補を除外する。
        """
        if self.path is None:raise ValueError('Path未設定')
        if len(pose)!=3 or len(velocity)!=2 or not np.isfinite(np.r_[pose,velocity]).all():
            raise ValueError('poseまたは速度が不正です')
        if velocity[0]<-.01 or velocity[0]>self.c.max_speed+.01 or abs(velocity[1])>self.c.max_yaw_rate+.01:
            raise ValueError('現在速度が前進PoCのモデル範囲外です')
        # 車体実測と前回指令を区別する。応答係数<1のとき実測yawを指令履歴
        # とみなすと、毎周期の指令変化量が余計に制限され旋回が頭打ちになる。
        if previous_command is None:
            previous_command=velocity
        if len(previous_command)!=2 or not np.isfinite(previous_command).all():
            raise ValueError('前回指令が不正です')
        if previous_command[0]<-.01 or previous_command[0]>self.c.max_speed+.01 or abs(previous_command[1])>self.c.max_yaw_rate+.01:
            raise ValueError('前回指令がモデル範囲外です')
        c=self.c;projected,error=self._project(pose[0],pose[1],velocity[0])
        if error>c.max_path_error:raise ValueError('Pathからの逸脱が上限を超えています')
        self.station=max(self.station,projected)
        remaining=self.s[-1]-self.station
        if remaining<=c.goal_radius and math.hypot(pose[0]-self.path[-1,0],pose[1]-self.path[-1,1])<=c.goal_radius:
            self.nominal.fill(0.)
            return dict(command=(0.,0.),trajectory=np.array([pose]),trajectory_controls=np.zeros((1,2)),station=self.station,goal=True,cost=0.)
        # 有限horizon先にある参照点を弧長で補間。近接する復路を最近傍として拾わない。
        if c.use_reference_speed_profile:
            # 目標1 m/sを全区間に押し付けず、曲率と加減速から到達可能な
            # 弧長を順に予測する。先の直線も手前の減速・再加速を経て参照する。
            q=self.station;ref_v=max(0.,velocity[0]);targets=[];speeds=[]
            accel=c.max_yaw_accel*c.min_radius
            for _ in range(c.steps):
                desired=float(np.interp(q,self.s,self.reference_speed))
                ref_v=float(np.clip(desired,max(0.,ref_v-accel*c.dt),ref_v+accel*c.dt))
                q=min(self.s[-1],q+ref_v*c.dt)
                targets.append(min(self.s[-1],q+c.lookahead_m));speeds.append(ref_v)
            target_s=np.array(targets);target_speed=np.array(speeds)
        else:
            offsets=c.lookahead_m+np.arange(1,c.steps+1)*c.dt*c.target_speed
            target_s=np.minimum(self.s[-1],self.station+offsets)
            target_speed=np.full(c.steps,c.target_speed)
        # 最終GOALも必須到達。許容帯の横に止まったまま完了待ちにならないよう、
        # 予測が終点へ達する段階では終点のXYを明示的に追う。
        horizon=c.target_speed*c.steps*c.dt+c.goal_radius
        required_active=(self.required_station is not None and self.required_station-self.station<=horizon) or remaining<=horizon
        if self.required_station is not None:
            # 必須到達点より先の目標を追ってショートカットしない。
            target_s=np.minimum(target_s,self.required_station)
        tx=np.interp(target_s,self.s,self.path[:,0]);ty=np.interp(target_s,self.s,self.path[:,1])
        # 終点でtarget_sが重複しても、隣接Path区間の方向を保持する。
        indices=np.clip(np.searchsorted(self.s,target_s,side='right')-1,0,len(self.path)-2)
        directions=self.path[indices+1]-self.path[indices]
        target_yaw=np.arctan2(directions[:,1],directions[:,0])
        controls=None;cost=None
        candidate_trace=None
        for iteration in range(c.iterations):
            noise=self.rng.normal(size=(c.batch_size,c.steps,2))
            noise[:,:,0]*=c.sigma_v;noise[:,:,1]*=c.sigma_w
            controls=self.nominal[None,:,:]+noise
            # 一候補だけ従来解を必ず含め、乱数により全候補が悪化するのを抑える。
            controls[0]=self.nominal
            x=np.full(c.batch_size,pose[0]);y=np.full(c.batch_size,pose[1]);yaw=np.full(c.batch_size,pose[2])
            v=np.full(c.batch_size,max(0.,previous_command[0]));w=np.full(c.batch_size,previous_command[1])
            cost=np.zeros(c.batch_size);valid=np.ones(c.batch_size,dtype=bool)
            # 表示対象だけ保存する。全候補のPath／ROS messageは生成しない。
            if iteration==c.iterations-1 and c.candidate_sample_count:
                candidate_trace=np.empty((c.candidate_sample_count,c.steps,2),dtype=np.float64)
            for k in range(c.steps):
                # 現在位置の離隔不足に応じて速度上限を下げる。減速度制約を守り、
                # 境界への接近で上限が急変しても速度を瞬時に飛ばさない。
                speed_cap=c.max_speed
                if environment is not None:
                    risk=environment.clearance_risk(x,y,c.clearance_distance_m)
                    slow=min(c.clearance_min_speed_mps,c.target_speed)
                    speed_cap=np.minimum(c.max_speed,c.target_speed-(c.target_speed-slow)*risk)
                v=np.clip(controls[:,k,0],np.maximum(0.,v-c.max_yaw_accel*c.min_radius*c.dt),
                          np.maximum(np.maximum(0.,v-c.max_yaw_accel*c.min_radius*c.dt),np.minimum(speed_cap,v+c.max_yaw_accel*c.min_radius*c.dt)))
                w=np.clip(controls[:,k,1],w-c.max_yaw_accel*c.dt,w+c.max_yaw_accel*c.dt)
                # 両制約が両立しない減速時は、vehicle interface同様に半径・停止を優先する。
                # この場合yawの変化量が加速度上限を超え得るため、滑らかな走行軌道ではない。
                w=np.clip(w,-np.minimum(c.max_yaw_rate,v/c.min_radius),np.minimum(c.max_yaw_rate,v/c.min_radius))
                controls[:,k,0]=v;controls[:,k,1]=w
                # 指令制約は従来どおり適用し、車体の予測旋回だけ係数を掛ける。
                old_x=x;old_y=y
                yaw=yaw+w*c.yaw_response_gain*c.dt;x=x+v*np.cos(yaw)*c.dt;y=y+v*np.sin(yaw)*c.dt
                if candidate_trace is not None:
                    candidate_trace[:,k,0]=x[:c.candidate_sample_count]
                    candidate_trace[:,k,1]=y[:c.candidate_sample_count]
                delta_yaw=np.arctan2(np.sin(yaw-target_yaw[k]),np.cos(yaw-target_yaw[k]))
                # 距離[m²]、角度[rad²]、速度[m²/s²]、yaw rate[rad²/s²]を重み付きで加算。
                dx=x-tx[k];dy=y-ty[k]
                if c.tracking_mode=='corridor':
                    # 当該時刻の接線に直交する横ずれだけに許容幅を設ける。
                    # 縦方向は弱いcostで前進を誘導し、帯の端へ張り付く指令を強制しない。
                    along=dx*np.cos(target_yaw[k])+dy*np.sin(target_yaw[k])
                    cross=-dx*np.sin(target_yaw[k])+dy*np.cos(target_yaw[k])
                    tolerance=0. if required_active else c.path_tolerance_m
                    # 必須点・GOALへの接近は従来の重みを保ち、通常区間の帯外だけ弱く誘導する。
                    weight=c.path_cost_weight if required_active else c.corridor_cost_weight
                    cost+=weight*np.maximum(0.,np.abs(cross)-tolerance)**2+c.progress_cost_weight*along**2
                else:cost+=c.path_cost_weight*(dx*dx+dy*dy)
                cost+=c.heading_cost_weight*delta_yaw**2
                if environment is not None:
                    terrain=environment.segment(old_x,old_y,x,y)
                    valid&=terrain<100
                    cost+=c.terrain_cost_weight*(terrain/100.)**2
                    risk=environment.clearance_risk(x,y,c.clearance_distance_m)
                    cost+=c.clearance_cost_weight*risk*risk
                cost+=c.speed_cost_weight*(v-target_speed[k])**2+c.yaw_cost_weight*w*w
            if c.tracking_mode=='point' or required_active:
                cost+=c.terminal_cost_weight*((x-tx[-1])**2+(y-ty[-1])**2)
            else:
                # 許容帯の中でも進行方向には進む。帯外への迂回を無制限には許さない。
                along=(x-tx[-1])*math.cos(target_yaw[-1])+(y-ty[-1])*math.sin(target_yaw[-1])
                cost+=c.progress_cost_weight*along**2
            if not np.isfinite(cost).all():raise ValueError('候補costが非有限です')
            if not valid.any():raise NoValidTrajectoryError('hazard・unknown・map外を避ける候補軌道がありません')
            weights=np.zeros(c.batch_size)
            weights[valid]=np.exp(-(cost[valid]-np.min(cost[valid]))/c.temperature);weights/=np.sum(weights)
            self.nominal=np.einsum('n,ntc->tc',weights,controls)
        # 平均後にも制約と環境を再検査。左右に分かれた候補の平均が障害物を
        # 横切る場合は、最後の反復で実際に有効だった最小cost候補へ戻す。
        best=int(np.argmin(np.where(valid,cost,np.inf)))
        for plan in (self.nominal,controls[best]):
            x,y,yaw=map(float,pose);v=max(0.,float(previous_command[0]));w=float(previous_command[1]);traj=[];safe=[]
            for desired_v,desired_w in plan:
                speed_cap=c.max_speed
                if environment is not None:
                    risk=float(environment.clearance_risk(x,y,c.clearance_distance_m))
                    slow=min(c.clearance_min_speed_mps,c.target_speed)
                    speed_cap=min(c.max_speed,c.target_speed-(c.target_speed-slow)*risk)
                v=float(np.clip(desired_v,max(0.,v-c.max_yaw_accel*c.min_radius*c.dt),max(max(0.,v-c.max_yaw_accel*c.min_radius*c.dt),min(speed_cap,v+c.max_yaw_accel*c.min_radius*c.dt))))
                w=float(np.clip(desired_w,w-c.max_yaw_accel*c.dt,w+c.max_yaw_accel*c.dt))
                w=float(np.clip(w,-min(c.max_yaw_rate,v/c.min_radius),min(c.max_yaw_rate,v/c.min_radius)))
                safe.append((v,w));yaw+=w*c.yaw_response_gain*c.dt;x+=v*math.cos(yaw)*c.dt;y+=v*math.sin(yaw)*c.dt;traj.append((x,y,yaw))
            if environment is None:break
            previous_xy=np.vstack((np.asarray(pose[:2]),np.asarray(traj)[:-1,:2]))
            if np.all(environment.segment(previous_xy[:,0],previous_xy[:,1],np.asarray(traj)[:,0],np.asarray(traj)[:,1])<100):break
        else:raise NoValidTrajectoryError('採用軌道がhazardまたはunknownに接触します')
        command=safe[0];self.nominal=np.vstack((np.asarray(safe[1:]),np.asarray(safe[-1:])))
        if not np.isfinite(self.nominal).all():raise ValueError('指令が非有限です')
        return dict(command=command,trajectory=np.asarray(traj),trajectory_controls=np.asarray(safe),candidate_trajectories=candidate_trace,
                    station=self.station,goal=False,cost=float(np.min(cost[valid])))
