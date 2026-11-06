"""非駆動の環境認識確認パネル。変更はROS parameter serviceの応答後に反映表示する。"""
import sys
import time
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtWidgets import (QApplication,QWidget,QVBoxLayout,QFormLayout,QDoubleSpinBox,
    QSpinBox,QPushButton,QLabel,QComboBox,QFileDialog,QScrollArea)
import yaml


# 上下限はproducerの実行中変更契約と同じ。単位を画面にも明記する。
FIELDS=(('map_size_x','map一辺 [m]',1.,30.,8.),
    ('resolution','セル幅 [m]',.05,.5,.1),
    ('min_depth','depth下限 [m]',.1,10.,.4),('max_depth','depth上限 [m]',.2,20.,7.),
    ('pixel_stride','pixel stride',1,16,4),
    ('hazard_obstacle_height_limit','obstacle閾値 [m]',.01,2.,.08),
    ('confirmation_window','解除窓 N [画像]',1,30,3),
    ('clear_min_points','解除の最低セル点数',1,1000,3),
    ('clear_count_ratio','解除の点数比',.01,1.,.75),
    ('clear_ground_tolerance','解除の最小z許容差 [m]',.001,1.,.03),
    ('hazard_slope_limit_deg','slope閾値 [deg]',1.,80.,20.),
    ('hazard_roughness_limit','roughness閾値 [m]',.001,1.,.03),
    ('hazard_step_limit','step閾値 [m]',.001,1.,.07))


CLEARANCE=('clearance_distance_m','clearance_cost_weight','clearance_min_speed_mps')


class SpeedPlot(QWidget):
    """採用軌道の速度を軽量描画する。候補軌道や画像の転送は行わない。"""
    def __init__(self):
        super().__init__();self.data={'valid':False};self.setMinimumHeight(180)

    def receive(self,message):
        try:self.data=json.loads(message.data)
        except (ValueError,TypeError):self.data={'valid':False}
        self.update()

    def paintEvent(self,event):
        painter=QPainter(self);painter.fillRect(self.rect(),QColor('#20252a'))
        painter.setPen(Qt.white)
        d=self.data
        if not d.get('valid'):
            painter.drawText(12,30,'有効な予測なし（速度グラフを消去）');return
        w=self.width()-80;h=self.height()-75
        xmax=max(.1,max(d['distance_m']));ymax=max(d['color_max_mps'],abs(d['actual_mps']),max(d['cap_mps']),.1)
        def point(x,y):return (int(48+x/xmax*w),int(35+h-y/ymax*h))
        painter.drawText(8,16,'速度 [m/s]   横軸：区間開始までの軌道距離 [m]')
        painter.drawText(8,self.height()-8,'赤 0 → 黄 → 緑 %.2f m/s（固定色範囲）'%d['color_max_mps'])
        painter.drawLine(48,35,48,35+h);painter.drawLine(48,35+h,48+w,35+h)
        painter.drawText(3,40,'%.1f'%ymax);painter.drawText(27,35+h,'0')
        painter.drawText(48+w-35,50+h,'%.2f'%xmax)
        # 速度上限と採用速度は同じ区間開始位置で比較し、実測速度は水平線で区別する。
        for values,color,style in (([d['target_mps']]*len(d['distance_m']),'#aaaaaa',Qt.DotLine),
                (d['cap_mps'],'#ffae42',Qt.DashLine),(d['speed_mps'],'#44e5de',Qt.SolidLine),
                ([d['actual_mps']]*len(d['distance_m']),'#ff78c9',Qt.DashDotLine)):
            painter.setPen(QPen(QColor(color),2,style))
            points=[point(x,y) for x,y in zip(d['distance_m'],values)]
            if len(points)==1:painter.drawEllipse(points[0][0]-2,points[0][1]-2,4,4)
            for a,b in zip(points,points[1:]):painter.drawLine(*a,*b)


class LabWindow(QWidget):
    def __init__(self,node):
        super().__init__();self.node=node;self.pending=None;self.loaded=False;self.widgets={}
        self.perception_loaded=False;self.preview_loaded=False
        self.setWindowTitle('Patasmonkey 環境認識 / 非駆動確認');self.resize(510,790)
        layout=QVBoxLayout(self)
        drive=node.get_parameter('drive_enabled').value
        self.offline=node.get_parameter('offline_mode').value if node.has_parameter('offline_mode') else False
        label=QLabel('オフラインbag確認：センサ・モーターは起動していません。' if self.offline else
                    'Joy操縦ON：車体を駆動できます。MPPIは表示のみです。' if drive else
                     '手押し用：モーターIDLE・状態取得のみ。MPPIは表示のみです。')
        label.setStyleSheet('color: #c04020; font-weight: bold;');layout.addWidget(label)
        self.status=QLabel('VIO・map待ち');self.status.setWordWrap(True);layout.addWidget(self.status)
        self.speed_plot=SpeedPlot();layout.addWidget(self.speed_plot)
        self.speed_summary=QLabel('速度待ち');self.speed_summary.setWordWrap(True);layout.addWidget(self.speed_summary)
        node.create_subscription(String,'/perception_preview/speed_profile',self.on_speed,1)
        self.result=QLabel('実効parameter読込待ち');self.result.setWordWrap(True);layout.addWidget(self.result)
        if self.offline:
            self.bag_status=QLabel('bag player待ち');layout.addWidget(self.bag_status)
            self.pause_client=node.create_client(SetBool,'/perception_bag_player/set_paused')
            self.rate_client=node.create_client(SetParametersAtomically,'/perception_bag_player/set_parameters_atomically')
            pause=QPushButton('bag一時停止');pause.clicked.connect(lambda:self.pause(True));layout.addWidget(pause)
            play=QPushButton('bag再生');play.clicked.connect(lambda:self.pause(False));layout.addWidget(play)
            self.rate=QDoubleSpinBox();self.rate.setRange(.1,4.);self.rate.setSingleStep(.1);self.rate.setValue(1.)
            self.rate_loaded=False
            layout.addWidget(self.rate)
            rate=QPushButton('再生速度を適用')
            rate.clicked.connect(lambda:self.send(self.rate_client,{'rate':self.rate.value()},
                lambda:self.result.setText('再生速度を反映しました')));layout.addWidget(rate)
            node.create_subscription(String,'/perception_bag_player/status',self.on_bag_status,1)
        content=QWidget();form=QFormLayout(content)
        for name,label,lo,hi,value in FIELDS:
            box=QSpinBox() if isinstance(value,int) else QDoubleSpinBox()
            if isinstance(box,QDoubleSpinBox):box.setDecimals(3);box.setSingleStep(.01)
            box.setRange(lo,hi);box.setValue(value);self.widgets[name]=box;form.addRow(label,box)
        self.mode=QComboBox();self.mode.addItem('obstacleのみ','obstacle_only')
        self.mode.addItem('obstacle＋terrain','full_terrain');form.addRow('表示・予測に使うcost',self.mode)
        self.reference_mode=QComboBox();self.reference_mode.addItem('初期位置の直線を固定','fixed_start')
        self.reference_mode.addItem('現在位置・向きから毎回直進','current_pose')
        if self.offline:self.reference_mode.setCurrentIndex(1)
        form.addRow('直進参照の作り方',self.reference_mode)
        self.length=QDoubleSpinBox();self.length.setRange(1.,30.);self.length.setValue(8.);form.addRow('直進参照の長さ [m]',self.length)
        self.speed=QDoubleSpinBox();self.speed.setRange(.05,1.);self.speed.setValue(1.)
        self.speed.setEnabled(False);form.addRow('予測の目標速度 [m/s]',self.speed)
        self.horizon=QLabel('plannerの実効予測時間を読込中');form.addRow('内部予測窓',self.horizon)
        self.radius=QDoubleSpinBox();self.radius.setRange(.1,1.);self.radius.setDecimals(3)
        self.radius.setSingleStep(.01);self.radius.setValue(.3);self.radius.setEnabled(False)
        form.addRow('車体半径（円近似） [m]',self.radius)
        self.margin=QDoubleSpinBox();self.margin.setRange(0.,10.);self.margin.setDecimals(3)
        self.margin.setSingleStep(.01);self.margin.setValue(.1);self.margin.setEnabled(False)
        form.addRow('footprint追加余裕 [m]',self.margin)
        self.inflation=QLabel();form.addRow('入力値の膨張半径',self.inflation)
        self.radius.valueChanged.connect(self.update_footprint_label)
        self.margin.valueChanged.connect(self.update_footprint_label);self.update_footprint_label()
        self.clearance={}
        for name,label,maximum in zip(CLEARANCE,('希望clearance [m]','clearance cost重み','余裕不足時の速度上限 [m/s]'),(10.,100.,1.)):
            box=QDoubleSpinBox();box.setDecimals(3);box.setRange(0.,maximum);box.setEnabled(False)
            self.clearance[name]=box;form.addRow(label,box)
        scroll=QScrollArea();scroll.setWidgetResizable(True);scroll.setWidget(content);layout.addWidget(scroll)
        self.apply_button=QPushButton('適用（認識mapを消去して再観測）');self.apply_button.clicked.connect(self.apply)
        layout.addWidget(self.apply_button);self.apply_button.setEnabled(False)
        preview=QPushButton('cost切替・予測設定だけ適用（mapは保持）')
        preview.clicked.connect(self.apply_preview);layout.addWidget(preview)
        reset=QPushButton('認識mapだけ消去');reset.clicked.connect(lambda:self.trigger('/depth_elevation_mapper/reset_map'));layout.addWidget(reset)
        reference=QPushButton('現在の位置・向きから直進参照を再設定')
        reference.clicked.connect(lambda:self.trigger('/perception_preview/reset_reference'));layout.addWidget(reference)
        save=QPushButton('画面の認識設定を別YAMLへ保存');save.clicked.connect(self.save);layout.addWidget(save)
        note=QLabel('unknownは安全値に変換しません。車体直下の未観測域などで予測不能なら、\n'
                    '明示した理由を表示します。適用値は実行中だけ有効、永続化は別YAML保存です。')
        note.setWordWrap(True);layout.addWidget(note)
        self.get=node.create_client(GetParameters,'/depth_elevation_mapper/get_parameters')
        self.preview_get=node.create_client(GetParameters,'/perception_preview/get_parameters')
        self.set=node.create_client(SetParametersAtomically,'/depth_elevation_mapper/set_parameters_atomically')
        self.plan=node.create_client(SetParametersAtomically,'/perception_preview/set_parameters_atomically')
        self.triggers={}
        node.create_subscription(String,'/perception_preview/status',lambda msg:self.status.setText(msg.data),1)
        self.timer=QTimer(self);self.timer.timeout.connect(self.poll);self.timer.start(30)

    def poll(self):
        # GUI threadを長時間占有しない。MPPI計算は別ROS processで行う。
        until=time.monotonic()+.004
        for _ in range(5):
            rclpy.spin_once(self.node,timeout_sec=0.)
            if time.monotonic()>until:break
        if not self.perception_loaded and self.pending is None and self.get.service_is_ready():
            req=GetParameters.Request();req.names=[row[0] for row in FIELDS]
            future=self.get.call_async(req);self.pending=future
            def done(f):
                self.pending=None
                try:
                    values=f.result().values
                    for (name,*_),value in zip(FIELDS,values):
                        self.widgets[name].setValue(Parameter.from_parameter_msg(
                            self._parameter_message(name,value)).value)
                    self.perception_loaded=True;self.result.setText('認識parameter読込済み、planner設定を取得中')
                except Exception as exc:self.result.setText(str(exc))
            future.add_done_callback(done)
        if self.perception_loaded and not self.preview_loaded and self.pending is None and self.preview_get.service_is_ready():
            req=GetParameters.Request();req.names=['target_speed','prediction_steps','prediction_dt',
                'reference_mode','mode','reference_length_m','footprint_radius_m','footprint_margin_m']+list(CLEARANCE)
            future=self.preview_get.call_async(req);self.pending=future
            def preview_done(f):
                self.pending=None
                try:
                    values={name:Parameter.from_parameter_msg(self._parameter_message(name,value)).value
                            for name,value in zip(req.names,f.result().values)}
                    for name,box in self.clearance.items():box.setValue(values[name]);box.setEnabled(True)
                    self.speed.setValue(values['target_speed']);self.speed.setEnabled(True)
                    self.length.setValue(values['reference_length_m'])
                    self.radius.setValue(values['footprint_radius_m']);self.radius.setEnabled(True)
                    self.margin.setValue(values['footprint_margin_m']);self.margin.setEnabled(True)
                    self.mode.setCurrentIndex(self.mode.findData(values['mode']))
                    self.reference_mode.setCurrentIndex(self.reference_mode.findData(values['reference_mode']))
                    self.horizon.setText('%.2f秒 = %d steps × %.3f秒（表示更新とは別）'%
                        (values['prediction_steps']*values['prediction_dt'],values['prediction_steps'],values['prediction_dt']))
                    self.preview_loaded=True;self.loaded=True;self.apply_button.setEnabled(True)
                    self.result.setText('認識・plannerの実効parameterを読み込みました')
                except Exception as exc:self.result.setText(str(exc))
            future.add_done_callback(preview_done)

    @staticmethod
    def _parameter_message(name,value):
        from rcl_interfaces.msg import Parameter as ParameterMessage
        msg=ParameterMessage();msg.name=name;msg.value=value;return msg

    def update_footprint_label(self):
        """編集中の入力値の和を表示。nodeへの反映は適用ボタンの応答で確認する。"""
        radius=self.radius.value();margin=self.margin.value()
        self.inflation.setText('%.3f m = %.3f + %.3f'%(radius+margin,radius,margin))

    def values(self):
        values={name:box.value() for name,box in self.widgets.items()}
        values['map_size_y']=values['map_size_x'];return values

    def send(self,client,values,callback):
        if self.pending is not None:self.result.setText('前の要求の応答待ちです');return
        if not client.service_is_ready():self.result.setText('対象nodeのserviceがありません');return
        req=SetParametersAtomically.Request()
        req.parameters=[Parameter(name,value=value).to_parameter_msg() for name,value in values.items()]
        self.apply_button.setEnabled(False);future=client.call_async(req);self.pending=future
        def done(f):
            self.pending=None;self.apply_button.setEnabled(self.loaded)
            try:
                result=f.result().result
                if not result.successful:self.result.setText('変更拒否: '+result.reason);return
                callback()
            except Exception as exc:self.result.setText(str(exc))
        future.add_done_callback(done)

    def apply(self):
        """producer成功後にpreviewを変更。二node間のtransactionとは扱わない。"""
        plan=dict(mode=self.mode.currentData(),reference_mode=self.reference_mode.currentData(),reference_length_m=self.length.value(),
                  target_speed=self.speed.value(),footprint_radius_m=self.radius.value(),footprint_margin_m=self.margin.value(),**{k:v.value() for k,v in self.clearance.items()})
        def perception_done():
            self.result.setText('認識設定反映・map消去済み。preview設定を反映中')
            self.send(self.plan,plan,lambda:self.result.setText('認識・preview設定を反映しました'))
        self.send(self.set,self.values(),perception_done)

    def apply_preview(self):
        self.send(self.plan,dict(mode=self.mode.currentData(),reference_mode=self.reference_mode.currentData(),reference_length_m=self.length.value(),
            target_speed=self.speed.value(),footprint_radius_m=self.radius.value(),footprint_margin_m=self.margin.value(),**{k:v.value() for k,v in self.clearance.items()}),
            lambda:self.result.setText('表示cost・予測設定を反映しました（認識mapは保持）'))

    def on_speed(self,message):
        """失敗通知でも表示を更新し、直前の成功速度を残さない。"""
        self.speed_plot.receive(message);d=self.speed_plot.data
        if not d.get('valid'):self.speed_summary.setText('有効な計画速度なし');return
        self.speed_summary.setText('水色：採用速度 %.2f → %.2f m/s / 桃：実測 %.2f m/s\n'
            '橙破線：距離に応じた上限 / 灰点線：設定目標 %.2f m/s'%
            (d['speed_mps'][0],d['speed_mps'][-1],d['actual_mps'],d['target_mps']))

    def on_bag_status(self,message):
        try:
            data=json.loads(message.data)
            self.bag_status.setText('bag: %s / %.1f s / %.2f倍速' %
                (data['state'],data['elapsed_sec'],data['rate']))
            if not self.rate_loaded:self.rate.setValue(data['rate']);self.rate_loaded=True
        except (ValueError,KeyError,TypeError):pass

    def pause(self,value):
        if not self.pause_client.service_is_ready():self.result.setText('bag player待ち');return
        req=SetBool.Request();req.data=value
        self.pause_client.call_async(req)

    def trigger(self,name):
        if self.pending is not None:self.result.setText('前の要求の応答待ちです');return
        client=self.triggers.setdefault(name,self.node.create_client(Trigger,name)) if name not in self.triggers else self.triggers[name]
        if not client.service_is_ready():self.result.setText('service待ち: '+name);return
        future=client.call_async(Trigger.Request());self.pending=future
        def done(f):
            self.pending=None
            try:self.result.setText(f.result().message)
            except Exception as exc:self.result.setText(str(exc))
        future.add_done_callback(done)

    def save(self):
        """正本を自動上書きせず、画面の認識値をユーザー指定ファイルへ保存する。"""
        name,_=QFileDialog.getSaveFileName(self,'認識設定を保存','perception_lab.yaml','YAML (*.yaml)')
        if name:
            values=self.values();values.update(depth_subscription_reliability='reliable',
                depth_subscription_queue_depth=3,debug_publish_rate=2.,allow_fallback_intrinsics=False)
            try:
                Path(name).write_text(yaml.safe_dump({'/**':{'ros__parameters':values}},allow_unicode=True))
                self.result.setText('画面の認識値を保存: '+name)
            except OSError as exc:self.result.setText(str(exc))


def main(args=None):
    rclpy.init(args=args);app=QApplication(sys.argv);node=Node('perception_lab_panel')
    node.declare_parameter('drive_enabled',False)
    node.declare_parameter('offline_mode',False)
    window=LabWindow(node);window.show()
    try:app.exec_()
    finally:window.timer.stop();node.destroy_node();rclpy.shutdown()
