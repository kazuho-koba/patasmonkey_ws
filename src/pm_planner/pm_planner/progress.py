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
        self.last_pose=None;self.deadline=None;self.completed=[]

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
        radius=self.pass_radius if event['attribute']=='pass' else self.stop_radius
        end=min(self.reference.length,self.station+self.max_advance,event['station']+radius)
        distance,station=project_local(self.reference,x,y,max(0.,self.station-.2),end)
        if distance>self.max_deviation:
            self.state='FAULT';self.reason='経路逸脱';return
        if station-self.station>allowed:
            self.state='FAULT';self.reason='経路弧長の不自然なジャンプ';return
        self.station=max(self.station,station)
        # 距離円だけでは往路で近い復路の点を誤消化し得るため、経路上で対象点の
        # 直前まで進んだ条件も要求する。.15mは到達を早めてよい弧長の余裕。
        near=math.hypot(x-event['x'],y-event['y'])<=radius
        arrived=self.station>=max(0.,event['station']-.15) and near
        if not near and self.station>=event['station']+radius-.001:
            self.state='FAULT';self.reason='対象waypointを未確認のまま通過';return
        if event['attribute']=='pass':
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
                    target_index=self.index,completed_waypoint_ids=list(self.completed))
