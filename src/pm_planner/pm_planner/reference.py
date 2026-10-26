"""直線・通常fillet・対称な前進回り込みによる非駆動参照Path。

PASSの小さな角はfillet、鋭角部は元点を通る3円弧で接続する。
回廊・perceptionは参照しない。停止属性の折曲・隣接接続の重複は拒否する。
衝突・敷地境界・曲率変化率の検証までは行わない。
"""
from dataclasses import dataclass,field
import math


@dataclass(frozen=True)
class Limits:
    """半径m・速度m/s・角速度rad/s・角加速度rad/s²。共有車両設定を利用する。"""
    radius: float
    max_speed: float
    max_yaw_rate: float
    max_yaw_accel: float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in vars(self).values()) or self.radius<=0 or self.max_speed<=0:
            raise ValueError('車両制約は有限、半径・速度は正値が必要です')
        if self.max_yaw_rate<0 or self.max_yaw_accel<0:raise ValueError('yaw制約は非負値です')


@dataclass
class Reference:
    """samples=(x,y,yaw,s[m])、eventsは元waypointとパス弧長の対応。"""
    samples: list
    events: list
    limits: Limits
    spacing: float
    turns: list = field(default_factory=list)

    @property
    def length(self):return self.samples[-1][3]


def corner_geometry(vertex,u,v,theta,radius,bulb):
    """進入と離脱を角の二等分方向に対して対称に接続する。

    bulbは外向き→主旋回→外向きの3円弧。h=|theta|/2として
    alpha=acos((1+cos(h))/2)、d=R*(sin(h)+2*sin(alpha))とすると、
    進入点B-d*uから離脱点B+d*vまでを等半径で接続でき、弧長中点は
    元waypoint Bそのものになる。theta→piでもdは有限（R*(1+sqrt(3))）。
    """
    sign=1 if theta>0 else -1
    if bulb:
        half=abs(theta)/2
        alpha=math.acos(max(-1.,min(1.,(1+math.cos(half))/2)))
        trim=radius*(math.sin(half)+2*math.sin(alpha))
        sweeps=(-sign*alpha,sign*(abs(theta)+2*alpha),-sign*alpha)
    else:
        trim=radius*math.tan(abs(theta)/2);sweeps=(theta,)
    entry=(vertex[0]-trim*u[0],vertex[1]-trim*u[1])
    exit=(vertex[0]+trim*v[0],vertex[1]+trim*v[1])
    x,y=entry;yaw=math.atan2(u[1],u[0]);arcs=[]
    for sweep in sweeps:
        direction=1 if sweep>0 else -1
        center=(x-direction*radius*math.sin(yaw),y+direction*radius*math.cos(yaw))
        start=yaw-direction*math.pi/2
        arcs.append((center,start,sweep))
        x=center[0]+radius*math.cos(start+sweep)
        y=center[1]+radius*math.sin(start+sweep);yaw+=sweep
    if math.hypot(x-exit[0],y-exit[1])>1e-7:
        raise ValueError('回り込み接続の数値誤差が上限を超えています')
    return dict(entry=entry,exit=exit,trim=trim,arcs=arcs,theta=theta,
                mode='symmetric_bulb' if bulb else 'fillet')


def build_reference(points, waypoints, limits, spacing=.1, corner_allowance=.3, pass_radius=.5,
                    max_segment=500., max_samples=100000,sharp_turn_strategy='symmetric_bulb',
                    uturn_direction='counterclockwise'):
    """metric XY点列から、最小半径を守るline／arc列を作る。

    corner_allowanceは元PASS点から円弧への最大離隔。走行可能回廊の認識ではなく
    設定された幾何許容にすぎない。超過時は既定で対称3円弧へ切替え、
    回廊による制限を入れない。rejectを指定すれば従来の拒否方針になる。
    PAUSE/CONFIRMの位置は丸めて移動しない。
    """
    if len(points)!=len(waypoints) or len(points)<2:raise ValueError('2点以上の対応点列が必要です')
    if not all(math.isfinite(v) for p in points for v in p):raise ValueError('経路座標が非有限です')
    if not all(math.isfinite(v) for v in (spacing,corner_allowance,pass_radius,max_segment)) or spacing<=0 or corner_allowance<0 or pass_radius<=0 or max_segment<=0:
        raise ValueError('補間間隔等の設定範囲が不正です')
    if sharp_turn_strategy not in ('symmetric_bulb','reject'):raise ValueError('sharp_turn_strategyが不正です')
    if uturn_direction not in ('counterclockwise','clockwise'):raise ValueError('uturn_directionが不正です')
    vectors=[]; lengths=[]
    for a,b in zip(points,points[1:]):
        d=math.hypot(b[0]-a[0],b[1]-a[1])
        if d<.01:raise ValueError('重複・1cm未満の近接waypointがあります')
        if d>max_segment:raise ValueError('経路区間がmax_segment_mを超えています')
        vectors.append(((b[0]-a[0])/d,(b[1]-a[1])/d));lengths.append(d)
    corners={}; trims=[0.]*len(points)
    for i in range(1,len(points)-1):
        u,v=vectors[i-1],vectors[i]
        theta=math.atan2(u[0]*v[1]-u[1]*v[0],u[0]*v[0]+u[1]*v[1])
        if abs(theta)<1e-5:continue
        if waypoints[i]['attribute']!='pass':raise ValueError('停止属性waypointの折曲は未対応です: '+waypoints[i]['waypoint_id'])
        # 正確な180度だけはXYから左右を一意に決められないので明示設定を使う。
        # それ以外はincoming×outgoingの符号を保持し、左右を勝手に変更しない。
        if abs(abs(theta)-math.pi)<1e-8:
            theta=math.copysign(math.pi,1 if uturn_direction=='counterclockwise' else -1)
        # 接点までの切詰め長R*tan(|theta|/2)、元頂点と円弧の最短距離
        # R*(sec(theta/2)-1)を別々に検査する。半径だけを守って遠くへ迂回しない。
        near_uturn=abs(theta)>math.pi-.01
        deviation=math.inf if near_uturn else limits.radius*(1/math.cos(theta/2)-1)
        bulb=near_uturn or deviation>min(corner_allowance,pass_radius)
        if bulb and sharp_turn_strategy=='reject':
            if near_uturn:raise ValueError('ほぼ180度の折返しは単純円弧で接続できません')
            raise ValueError('waypoint %d: %.2f度の折曲には約%.3fmの離隔が必要（許容%.3fm）'%(
                i+1,math.degrees(abs(theta)),deviation,min(corner_allowance,pass_radius)))
        corners[i]=corner_geometry(points[i],u,v,theta,limits.radius,bulb)
        trims[i]=corners[i]['trim']
    for i,length in enumerate(lengths):
        if trims[i]+trims[i+1]>=length-.001:raise ValueError('隣接円弧が重なります。waypoint間隔・半径を見直してください')
    samples=[];event_stations={0:0.};station=0.;turns=[]

    def append(x,y,yaw,s):
        if len(samples)>=max_samples:raise ValueError('補間点数が上限を超えています')
        if samples and abs(s-samples[-1][3])<1e-9:return
        samples.append((x,y,yaw,s))

    current=points[0]
    for i in range(1,len(points)):
        # まず次の接点まで直線を弧長spacing以下で刻み、続いて同じ弧長軸へ
        # 円弧を追加する。waypointの進捗位置は円弧の中点に対応付ける。
        end=corners[i]['entry'] if i in corners else points[i]
        distance=math.hypot(end[0]-current[0],end[1]-current[1]);yaw=math.atan2(end[1]-current[1],end[0]-current[0])
        pieces=max(1,int(math.ceil(distance/spacing)))
        for j in range(pieces+1):
            t=j/pieces;append(current[0]+t*(end[0]-current[0]),current[1]+t*(end[1]-current[1]),yaw,station+t*distance)
        station+=distance
        if i in corners:
            corner=corners[i];arcs=corner['arcs']
            total=sum(limits.radius*abs(a[2]) for a in arcs)
            event_stations[i]=station+total/2
            turns.append(dict(waypoint_index=i,waypoint_id=waypoints[i]['waypoint_id'],
                mode=corner['mode'],direction='counterclockwise' if corner['theta']>0 else 'clockwise',
                angle_rad=corner['theta'],radius_m=limits.radius,trim_m=corner['trim'],
                entry=corner['entry'],exit=corner['exit'],start_s=station,end_s=station+total,
                event_s=station+total/2))
            for k,(center,start,sweep) in enumerate(arcs):
                arc=limits.radius*abs(sweep);pieces=max(2,int(math.ceil(arc/spacing)))
                # 3円弧の中央円弧を偶数分割して、元waypointをPath上にも必ず残す。
                if len(arcs)==3 and k==1 and pieces%2:pieces+=1
                for j in range(1,pieces+1):
                    angle=start+sweep*j/pieces;heading=angle+math.copysign(math.pi/2,sweep)
                    append(center[0]+limits.radius*math.cos(angle),center[1]+limits.radius*math.sin(angle),heading,station+arc*j/pieces)
                station+=arc
            current=corner['exit']
        else:
            event_stations[i]=station;current=points[i]
    events=[dict(w, x=points[i][0],y=points[i][1],station=event_stations[i]) for i,w in enumerate(waypoints)]
    return Reference(samples,events,limits,spacing,turns)


def project_local(reference,x,y,start_s,end_s):
    """現在の弧長区間内だけで最近傍を探す。ループの離れた区間へ飛ばない。"""
    best=None
    for a,b in zip(reference.samples,reference.samples[1:]):
        if b[3]<start_s or a[3]>end_s:continue
        dx,dy=b[0]-a[0],b[1]-a[1];length2=dx*dx+dy*dy
        t=0. if length2==0 else ((x-a[0])*dx+(y-a[1])*dy)/length2
        # 探索窓の端を跨ぐ補間区間も、許可した弧長区間内に投影先を制限する。
        lo=max(0.,(start_s-a[3])/(b[3]-a[3]));hi=min(1.,(end_s-a[3])/(b[3]-a[3]))
        t=max(lo,min(hi,t));px,py=a[0]+t*dx,a[1]+t*dy
        candidate=(math.hypot(x-px,y-py),a[3]+t*(b[3]-a[3]))
        if best is None or candidate<best:best=candidate
    if best is None:raise ValueError('投影可能な参照区間がありません')
    return best
