"""位置入力に対するミッション進捗の非駆動状態機械。

STOP_REQUIREDは速度指令ではなく将来controllerへの要求状態。車両が止まったとは
推定せず、停止点付近で低速が観測された後に待機時間・確認・完了へ進む。
"""
import math
from .reference import project_local


class Progress:
    def __init__(self,reference,pass_radius=.5,stop_radius=.25,start_radius=.5,
                 max_deviation=1.,max_advance=2.,jump_slack=.3,max_pose_gap=1.,stop_speed=.03,
                 heading_tolerance=.52):
        values=(pass_radius,stop_radius,start_radius,max_deviation,max_advance,jump_slack,max_pose_gap,stop_speed,heading_tolerance)
        if not all(math.isfinite(v) and v>0 for v in values):raise ValueError('進捗parameterは正の有限値が必要です')
        self.reference=reference;self.pass_radius=pass_radius;self.stop_radius=stop_radius
        self.start_radius=start_radius;self.max_deviation=max_deviation;self.max_advance=max_advance
        self.jump_slack=jump_slack;self.max_pose_gap=max_pose_gap;self.stop_speed=stop_speed
        self.heading_tolerance=heading_tolerance
        self.state='READY';self.reason='非駆動preview待機';self.station=0.;self.index=0
        self.last_pose=None;self.deadline=None;self.completed=[];self.inferred_completed=[]

    def start(self,pose):
        """現在pose=(stamp秒,x,y,yaw,speed)がSTART付近・停止・向き整合なら追跡開始。"""
        if self.state!='READY':raise ValueError('READY状態でのみ開始できます')
        stamp,x,y,yaw,speed=pose
        if not all(math.isfinite(v) for v in pose):raise ValueError('poseが非有限です')
        start=self.reference.samples[0]
        if math.hypot(x-start[0],y-start[1])>self.start_radius:raise ValueError('STARTから離れています。自動接続は行いません')
        difference=math.atan2(math.sin(yaw-start[2]),math.cos(yaw-start[2]))
        if abs(difference)>self.heading_tolerance:raise ValueError('START進入方向と車体headingが一致しません')
        if abs(speed)>self.stop_speed:raise ValueError('停止確認が必要です')
        self.index=1;self.completed=[self.reference.events[0]['waypoint_id']]
        self.last_pose=pose;self.state='RUNNING';self.reason='位置入力の追跡中（速度出力なし）'

    def pause(self,reason='operator pause'):
        if self.state in ('RUNNING','STOP_REQUIRED','WAITING_PAUSE','WAITING_CONFIRM'):
            self.state='MANUAL_PAUSED';self.reason=reason
            # stale中の時間経過をPAUSE完了に流用しない。再開後に待機をやり直す。
            self.deadline=None

    def resume(self,pose):
        if self.state!='MANUAL_PAUSED':raise ValueError('MANUAL_PAUSED状態ではありません')
        if not all(math.isfinite(v) for v in pose):raise ValueError('poseが非有限です')
        if abs(pose[4])>self.stop_speed:raise ValueError('再開には停止確認が必要です')
        distance,_=project_local(self.reference,pose[1],pose[2],max(0.,self.station-.2),
                                 min(self.reference.length,self.station+self.max_advance))
        if distance>self.max_deviation:raise ValueError('参照経路から離れています')
        self.last_pose=pose;self.state='RUNNING';self.reason='明示再開（速度出力なし）'

    def rejoin(self,pose,ambiguity_distance=.15,ambiguity_station=1.):
        """手動中断後だけ前方経路へ再投影し、飛ばしたPASSを推定通過とする。

        停止前の弧長より0.2 m以上後ろへは戻さない。距離だけで近接往復路を
        選ばないよう接線方向も照合する。同程度に近い別区間が残れば拒否する。
        全検査が成功するまで進捗を変更せず、GOALや停止属性を暗黙に消化しない。
        """
        if self.state!='MANUAL_PAUSED':raise ValueError('MANUAL_PAUSED状態ではありません')
        if not all(math.isfinite(v) for v in pose):raise ValueError('poseが非有限です')
        if abs(pose[4])>self.stop_speed:raise ValueError('再開には停止確認が必要です')
        candidates=[];near=False
        for a,b in zip(self.reference.samples,self.reference.samples[1:]):
            if b[3]<max(0.,self.station-.2):continue
            dx,dy=b[0]-a[0],b[1]-a[1];ds=b[3]-a[3]
            lo=max(0.,(self.station-.2-a[3])/ds)
            t=max(lo,min(1.,((pose[1]-a[0])*dx+(pose[2]-a[1])*dy)/(dx*dx+dy*dy)))
            distance=math.hypot(pose[1]-a[0]-t*dx,pose[2]-a[1]-t*dy)
            if distance>self.max_deviation:continue
            near=True;heading=math.atan2(dy,dx)
            error=math.atan2(math.sin(pose[3]-heading),math.cos(pose[3]-heading))
            if abs(error)<=self.heading_tolerance:candidates.append((distance,a[3]+t*ds))
        if not candidates:raise ValueError('経路の進行方向と車体headingが一致しません' if near else '参照経路から離れています')
        distance,station=min(candidates)
        end=self.reference.samples[-1]
        # 終点を行き過ぎた車体へ、暗黙の折返し・後退を作らない。
        beyond=(pose[1]-end[0])*math.cos(end[2])+(pose[2]-end[1])*math.sin(end[2])
        if station>=self.reference.length-1e-6 and beyond>0 and distance>self.stop_radius:
            raise ValueError('GOALを行き過ぎています。終点付近へ戻してから再開してください')
        if any(d<=distance+ambiguity_distance and abs(s-station)>ambiguity_station for d,s in candidates):
            raise ValueError('再開先の経路区間が曖昧です。交差・近接区間から離れてください')
        station=max(self.station,station);index=self.index
        # PASSの推定消化は通常の到達判定と区別して記録する。GOALは停止判定を残す。
        while index<len(self.reference.events)-1 and self.reference.events[index]['station']<station-1e-6:
            event=self.reference.events[index]
            if event['attribute']!='pass':
                # 必須点そのものの近くでの再開は許し、次のupdateで到達を確認する。
                if event['attribute']=='visit' and math.hypot(pose[1]-event['x'],pose[2]-event['y'])<=self.pass_radius:break
                raise ValueError('未実行の必須点・停止属性を飛ばして再開できません')
            index+=1
        inferred=[e['waypoint_id'] for e in self.reference.events[self.index:index]]
        self.completed.extend(inferred);self.inferred_completed.extend(inferred)
        self.index=index;self.station=station;self.last_pose=pose;self.deadline=None
        self.state='RUNNING';self.reason='現在位置へ進捗を合わせて明示再開'

    def confirm(self):
        if self.state!='WAITING_CONFIRM':raise ValueError('オペレータ確認待ちではありません')
        self._complete_event()

    def cancel(self):
        """参照経路は表示用に保持し、進捗を終了する。再実行にはreloadする。"""
        self.state='CANCELED';self.reason='明示cancel（速度出力なし）';self.deadline=None

    def _complete_event(self):
        event=self.reference.events[self.index];self.completed.append(event['waypoint_id'])
        self.index+=1;self.deadline=None
        self.state='COMPLETE' if self.index==len(self.reference.events) else 'RUNNING'
        self.reason='preview完了（車体を駆動していません）' if self.state=='COMPLETE' else '次のwaypointを追跡'

    def update(self,pose):
        """対象waypointまでの局所区間で投影し、順序を守って一つずつ消化する。"""
        if self.state not in ('RUNNING','STOP_REQUIRED','WAITING_PAUSE','WAITING_CONFIRM'):return
        if not all(math.isfinite(v) for v in pose):self.state='FAULT';self.reason='非有限pose';return
        stamp,x,y,yaw,speed=pose;old=self.last_pose;dt=stamp-old[0]
        if dt<=0:
            if dt<0:self.state='FAULT';self.reason='pose時刻の巻戻り'
            return
        allowed=self.reference.limits.max_speed*dt+self.jump_slack
        if dt>self.max_pose_gap or math.hypot(x-old[1],y-old[2])>allowed:
            self.state='FAULT';self.reason='pose欠落／ジャンプ。明示resetが必要';return
        self.last_pose=pose
        event=self.reference.events[self.index]
        radius=self.pass_radius if event['attribute'] in ('pass','visit') else self.stop_radius
        # 迂回やカーブ内側の通過では、最近傍の弧長が急に先へ移ることがある。
        # 探索窓自体を許容前進量へ制限し、進捗を段階的に追従させる。
        # 実poseのジャンプ検査は上で独立に行っており、近接復路への大ジャンプは許さない。
        advance=min(self.max_advance,allowed)
        end=min(self.reference.length,self.station+advance,event['station']+radius)
        distance,station=project_local(self.reference,x,y,max(0.,self.station-.2),end)
        if distance>self.max_deviation:
            self.state='FAULT';self.reason='経路逸脱';return
        if station-self.station>allowed+1e-8:
            self.state='FAULT';self.reason='経路弧長の不自然なジャンプ';return
        self.station=max(self.station,station)
        # 距離円だけでは往路で近い復路の点を誤消化し得るため、経路上で対象点の
        # 直前まで進んだ条件も要求する。.15mは到達を早めてよい弧長の余裕。
        near=math.hypot(x-event['x'],y-event['y'])<=radius
        # PASSは距離円へ入らなくても、許容帯内で経路弧長を越えれば通過。
        # .15 mの早期到達余裕は近接が必要なVISIT/停止属性だけへ残す。
        arrived=(self.station>=event['station'] if event['attribute']=='pass' else
                 self.station>=max(0.,event['station']-.15) and near)
        if event['attribute']!='pass' and not near and self.station>=event['station']+radius-.001:
            self.state='FAULT';self.reason='対象waypointを未確認のまま通過';return
        if event['attribute'] in ('pass','visit'):
            if arrived:self._complete_event()
            return
        if not arrived:
            # 停止場所から離れた時間をPAUSEへ加算せず、離れたままCONFIRMもしない。
            self.deadline=None
            if self.state in ('WAITING_PAUSE','WAITING_CONFIRM'):
                self.state='STOP_REQUIRED';self.reason='停止点から離れました。再到達・停止を確認してください'
            return
        if arrived:
            if abs(speed)>self.stop_speed:
                self.deadline=None
                self.state='STOP_REQUIRED';self.reason='停止位置付近。停止速度の観測待ち';return
            if event['attribute']=='goal':self._complete_event()
            elif event['attribute']=='confirm':
                self.state='WAITING_CONFIRM';self.reason='明示オペレータ確認待ち'
            elif event['attribute']=='pause':
                if self.deadline is None:self.deadline=stamp+event['pause_sec']
                self.state='WAITING_PAUSE';self.reason='停止後の待機時間を計測中'
                if stamp>=self.deadline:self._complete_event()

    def status(self):
        event=self.reference.events[self.index] if self.index<len(self.reference.events) else None
        return dict(state=self.state,reason=self.reason,progress_m=self.station,
                    target_waypoint_id=event['waypoint_id'] if event else None,
                    target_index=self.index,completed_waypoint_ids=list(self.completed),
                    inferred_completed_waypoint_ids=list(self.inferred_completed))
