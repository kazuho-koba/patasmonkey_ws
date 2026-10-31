"""非駆動MPPI PoC。NumPyで候補を一括計算し、環境cost・車両指令出力は持たない。

座標は同一のmetric frame、速度はm/s・rad/s。各制御stepで前進・速度・加速度・
最小半径を制約し、経路の前方弧長だけに沿って候補を評価する。実車用安全機能ではない。
"""
from dataclasses import dataclass
import math
import numpy as np


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
        for name in ('path_cost_weight','heading_cost_weight','speed_cost_weight','yaw_cost_weight','terminal_cost_weight'):
            if not math.isfinite(getattr(self,name)) or getattr(self,name)<0:
                raise ValueError(name+'は非負の有限値が必要です')


class MppiPoc:
    """1経路の進捗と前回最適入力を保持する。別経路には必ずset_pathでresetする。"""
    def __init__(self,config=MppiConfig()):
        self.c=config;self.rng=np.random.default_rng(config.seed)
        self.path=None;self.station=0.;self.nominal=np.zeros((config.steps,2),dtype=np.float64)

    def set_path(self,points):
        """Nx2のPathを検証し、弧長s[m]を付加する。重複点と非有限値は拒否する。"""
        p=np.asarray(points,dtype=np.float64)
        if p.ndim!=2 or p.shape[1]!=2 or len(p)<2 or not np.isfinite(p).all():
            raise ValueError('Pathは有限な2次元座標の2点以上が必要です')
        distance=np.linalg.norm(np.diff(p,axis=0),axis=1)
        if np.any(distance<1e-8):raise ValueError('Pathに重複点があります')
        self.path=p.copy();self.s=np.r_[0.,np.cumsum(distance)];self.station=0.
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

    def step(self,pose,velocity,previous_command=None):
        """pose=(x,y,yaw), velocity=(v,w)から次指令と採用予測軌道を求める。

        簡易MPPIの重み付き摂動更新を使う。候補は各stepで制約投影し、重みは
        最小costを差し引いたsoftmaxで算出する。障害物・unknownは評価しない。
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
            return dict(command=(0.,0.),trajectory=np.array([pose]),station=self.station,goal=True,cost=0.)
        # 有限horizon先にある参照点を弧長で補間。近接する復路を最近傍として拾わない。
        offsets=c.lookahead_m+np.arange(1,c.steps+1)*c.dt*c.target_speed
        target_s=np.minimum(self.s[-1],self.station+offsets)
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
            cost=np.zeros(c.batch_size)
            # 表示対象だけ保存する。全候補のPath／ROS messageは生成しない。
            if iteration==c.iterations-1 and c.candidate_sample_count:
                candidate_trace=np.empty((c.candidate_sample_count,c.steps,2),dtype=np.float64)
            for k in range(c.steps):
                v=np.clip(controls[:,k,0],np.maximum(0.,v-c.max_yaw_accel*c.min_radius*c.dt),
                          np.minimum(c.max_speed,v+c.max_yaw_accel*c.min_radius*c.dt))
                w=np.clip(controls[:,k,1],w-c.max_yaw_accel*c.dt,w+c.max_yaw_accel*c.dt)
                # 両制約が両立しない減速時は、vehicle interface同様に半径・停止を優先する。
                # この場合yawの変化量が加速度上限を超え得るため、滑らかな走行軌道ではない。
                w=np.clip(w,-np.minimum(c.max_yaw_rate,v/c.min_radius),np.minimum(c.max_yaw_rate,v/c.min_radius))
                controls[:,k,0]=v;controls[:,k,1]=w
                # 指令制約は従来どおり適用し、車体の予測旋回だけ係数を掛ける。
                yaw=yaw+w*c.yaw_response_gain*c.dt;x=x+v*np.cos(yaw)*c.dt;y=y+v*np.sin(yaw)*c.dt
                if candidate_trace is not None:
                    candidate_trace[:,k,0]=x[:c.candidate_sample_count]
                    candidate_trace[:,k,1]=y[:c.candidate_sample_count]
                delta_yaw=np.arctan2(np.sin(yaw-target_yaw[k]),np.cos(yaw-target_yaw[k]))
                # 距離[m²]、角度[rad²]、速度[m²/s²]、yaw rate[rad²/s²]を重み付きで加算。
                cost+=c.path_cost_weight*((x-tx[k])**2+(y-ty[k])**2)+c.heading_cost_weight*delta_yaw**2
                cost+=c.speed_cost_weight*(v-c.target_speed)**2+c.yaw_cost_weight*w*w
            cost+=c.terminal_cost_weight*((x-tx[-1])**2+(y-ty[-1])**2)
            if not np.isfinite(cost).all():raise ValueError('候補costが非有限です')
            weights=np.exp(-(cost-np.min(cost))/c.temperature);weights/=np.sum(weights)
            self.nominal=np.einsum('n,ntc->tc',weights,controls)
        # 更新後の平均も前回指令から逐次制約して出力・可視化を一致させる。
        x,y,yaw=map(float,pose);v=max(0.,float(previous_command[0]));w=float(previous_command[1]);traj=[];safe=[]
        for desired_v,desired_w in self.nominal:
            v=float(np.clip(desired_v,max(0.,v-c.max_yaw_accel*c.min_radius*c.dt),min(c.max_speed,v+c.max_yaw_accel*c.min_radius*c.dt)))
            w=float(np.clip(desired_w,w-c.max_yaw_accel*c.dt,w+c.max_yaw_accel*c.dt))
            w=float(np.clip(w,-min(c.max_yaw_rate,v/c.min_radius),min(c.max_yaw_rate,v/c.min_radius)))
            safe.append((v,w));yaw+=w*c.yaw_response_gain*c.dt;x+=v*math.cos(yaw)*c.dt;y+=v*math.sin(yaw)*c.dt;traj.append((x,y,yaw))
        command=safe[0];self.nominal=np.vstack((np.asarray(safe[1:]),np.asarray(safe[-1:])))
        if not np.isfinite(self.nominal).all():raise ValueError('指令が非有限です')
        return dict(command=command,trajectory=np.asarray(traj),candidate_trajectories=candidate_trace,
                    station=self.station,goal=False,cost=float(np.min(cost)))
