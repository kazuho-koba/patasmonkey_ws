"""環境認識review専用のclock付きbag player。制御topicは再生しない。

MCAP/SQLiteをstreamし、wall timerで再生を進める。GUIのpause/rate操作に応答し、
終端でもpublisher・最終clock・static TFを保持する。巻戻し・loopは推定器のresetが
必要なので扱わない。既存bag_clock_playerの挙動は変更しない。
"""
from collections import Counter
import json
import math
from pathlib import Path
import time
import sqlite3
import heapq

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rcl_interfaces.msg import SetParametersResult
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage
from .cli.bag_clock_player import MESSAGE_TYPES, iter_records


def bag_topics(bag):
    """messageが存在するtopicと型を読む。巨大な画像payloadは展開しない。"""
    import yaml
    path=Path(bag)/'metadata.yaml'
    if path.exists():
        info=yaml.safe_load(path.read_text())['rosbag2_bagfile_information']
        return {r['topic_metadata']['name']:r['topic_metadata']['type']
                for r in info['topics_with_message_count'] if r['message_count']>0}
    raise ValueError('metadata.yamlのあるbagディレクトリを指定してください')


def replay_topics(available,vio_source='auto'):
    """旧local TF・制御指令・既存hazardを除外し、EKFの入力だけを選ぶ。"""
    required=('/wheel/odometry','/wit/imu','/oak/depth/image_raw')
    missing=[name for name in required if name not in available]
    if missing:raise ValueError('再計算に必要なtopicがありません: '+', '.join(missing))
    if vio_source not in ('auto','raw','recorded'):raise ValueError('vio_sourceはauto/raw/recorded')
    raw='/ov_msckf/odomimu' in available and vio_source!='recorded'
    if vio_source=='raw' and not raw:raise ValueError('/ov_msckf/odomimuがありません')
    vio='/ov_msckf/odomimu' if raw else '/vio/odometry'
    if vio not in available:raise ValueError('VIO入力がありません: '+vio)
    chosen=list(required)+[vio]
    chosen += [name for name in ('/tf_static','/oak/depth/camera_info',
        '/oak/color/image_raw','/oak/color/camera_info') if name in available]
    return chosen,raw


def records(bag,topics):
    """MCAPは既存reader、SQLiteは標準sqlite3で選択topicをstreamする。"""
    path=Path(bag)
    if list(path.glob('*.mcap')):
        yield from iter_records(path,topics)
    elif list(path.glob('*.db3')):
        def read_database(file):
            # reader専用接続にして元bagを変更しない。Foxyにrosbag2_pyがない環境でも使える。
            db=sqlite3.connect(file.resolve().as_uri()+'?mode=ro',uri=True)
            try:
                placeholders=','.join('?' for _ in topics)
                query=('SELECT m.timestamp,t.name,m.data FROM messages m JOIN topics t ON t.id=m.topic_id '
                       'WHERE t.name IN ('+placeholders+') ORDER BY m.timestamp,m.id')
                yield from db.execute(query,topics)
            finally:db.close()
        # 分割DBの範囲が重なってもclockを逆行させない。一つずつ先読みするだけである。
        yield from heapq.merge(*(read_database(file) for file in path.glob('*.db3')),key=lambda row:row[0])
    else:raise ValueError('MCAPまたはSQLiteのbagがありません')


class PerceptionBagPlayer(Node):
    def __init__(self, **kwargs):
        super().__init__('perception_bag_player',**kwargs)
        for name,value in (('bag',''),('topics',list(('/wheel/odometry',))),('rate',1.),
                           ('start_paused',False),('startup_delay_sec',2.),('duration_sec',0.)):
            self.declare_parameter(name,value)
        self.rate=float(self.get_parameter('rate').value)
        if not math.isfinite(self.rate) or not .1<=self.rate<=4.:raise ValueError('rateは0.1〜4')
        self.topics=self.get_parameter('topics').value
        forbidden=set(self.topics)-set(MESSAGE_TYPES)
        if forbidden or '/tf' in self.topics or '/odometry/local' in self.topics:
            raise ValueError('review対象外topic: '+str(forbidden or self.topics))
        self.iterator=iter(records(self.get_parameter('bag').value,self.topics));self.next_record=next(self.iterator,None)
        if self.next_record is None:raise ValueError('選択messageがありません')
        self.first=self.next_record[0];self.cursor=self.first;self.last_stamp=self.first
        self.paused=self.get_parameter('start_paused').value;self.finished=False
        delay=self.get_parameter('startup_delay_sec').value
        if not math.isfinite(delay) or delay<0:raise ValueError('startup_delay_secは0以上')
        self.start=time.monotonic()+delay;self.last_wall=self.start
        self.duration=self.get_parameter('duration_sec').value
        if not math.isfinite(self.duration) or self.duration<0:raise ValueError('duration_secは0以上')
        self.counts=Counter();self.statics={};self.maximum_lateness=0.
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
        static=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pubs={name:self.create_publisher(MESSAGE_TYPES[name],name,static if name=='/tf_static' else qos)
                   for name in self.topics}
        self.clock=self.create_publisher(Clock,'/clock',qos)
        self.status=self.create_publisher(String,'~/status',qos)
        self.create_service(SetBool,'~/set_paused',self.set_paused)
        self.add_on_set_parameters_callback(self.configure)
        # player自身はuse_sim_time=false。止まった/clockを待つtimerにはしない。
        self.create_timer(.005,self.tick);self.create_timer(.5,self.publish_status)

    def configure(self,changes):
        """速度変更前の経過時間を旧rateで確定し、再生位置を飛ばさない。"""
        for p in changes:
            if p.name!='rate' or not isinstance(p.value,(int,float)) or not math.isfinite(p.value) or not .1<=p.value<=4.:
                return SetParametersResult(successful=False,reason='rateだけ0.1〜4で変更可能')
        self.advance();self.rate=float(changes[-1].value) if changes else self.rate
        return SetParametersResult(successful=True)

    def advance(self):
        now=time.monotonic()
        if now<self.start:return
        if not self.paused and not self.finished:self.cursor+=(now-self.last_wall)*self.rate*1e9
        self.last_wall=now

    def set_paused(self,request,response):
        self.advance();self.paused=request.data
        response.success=True;response.message='一時停止' if self.paused else '再生';return response

    def tick(self):
        self.advance()
        if time.monotonic()<self.start or self.paused or self.finished:return
        # 1tick当たりのburstを制限し、pause等のservice処理を飢餓にしない。
        for _ in range(30):
            if self.next_record is None:
                self.finished=True;self.publish_status();return
            stamp,topic,data=self.next_record
            if self.duration and stamp-self.first>self.duration*1e9:
                self.finished=True;self.publish_status();return
            if stamp>self.cursor:return
            if stamp<self.last_stamp:raise ValueError('bagの記録時刻が逆行しています')
            msg=deserialize_message(data,MESSAGE_TYPES[topic])
            if topic=='/tf_static':
                # late subscriberも固定TF全体を受け取れるよう、子frame毎に保持する。
                for tf in msg.transforms:
                    if tf.child_frame_id in ('base_link','odom','map'):continue
                    self.statics[tf.child_frame_id]=tf
                msg=TFMessage(transforms=list(self.statics.values()))
            clock=Clock();clock.clock.sec=int(stamp//10**9);clock.clock.nanosec=int(stamp%10**9)
            self.clock.publish(clock);self.pubs[topic].publish(msg)
            self.counts[topic]+=1;self.last_stamp=stamp
            self.maximum_lateness=max(self.maximum_lateness,(self.cursor-stamp)*1e-9/self.rate)
            self.next_record=next(self.iterator,None)

    def publish_status(self):
        self.status.publish(String(data=json.dumps(dict(state='finished' if self.finished else
            'paused' if self.paused else 'playing',rate=self.rate,
            elapsed_sec=(self.last_stamp-self.first)*1e-9,counts=dict(self.counts),
            maximum_lateness_sec=self.maximum_lateness),ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args);node=PerceptionBagPlayer()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
