"""review用の入力選択・pause・rate・終端維持を実デバイスなしで検証する。"""
import os
import json
import time
from pathlib import Path

import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool
from pm_evaluation import perception_bag_player as replay


def test_input_selection_never_replays_control_or_old_local_tf():
    available={name:'' for name in ('/wheel/odometry','/wit/imu','/oak/depth/image_raw',
        '/ov_msckf/odomimu','/vio/odometry','/tf','/cmd_vel','/motor_state','/tf_static')}
    topics,raw=replay.replay_topics(available)
    assert raw and '/ov_msckf/odomimu' in topics and '/vio/odometry' not in topics
    assert not set(topics)&{'/tf','/cmd_vel','/motor_state','/odometry/local'}
    assert not replay.replay_topics(available,'recorded')[1]
    del available['/wit/imu']
    with pytest.raises(ValueError,match='/wit/imu'):replay.replay_topics(available)


def test_missing_metadata_is_explicit(tmp_path):
    with pytest.raises(ValueError,match='metadata'):replay.bag_topics(tmp_path)


def test_sqlite_reader_is_ordered_and_topic_filtered(tmp_path):
    import sqlite3
    for number,stamps in enumerate(((10,30),(20,40))):
        db=sqlite3.connect(str(tmp_path/('bag_%d.db3'%number)))
        db.executescript('CREATE TABLE topics(id INTEGER,name TEXT);'
                         'CREATE TABLE messages(id INTEGER,topic_id INTEGER,timestamp INTEGER,data BLOB);')
        db.executemany('INSERT INTO topics VALUES(?,?)',[(1,'/wit/imu'),(2,'/cmd_vel')])
        db.executemany('INSERT INTO messages VALUES(?,?,?,?)',
            [(i,1,stamp,b'imu') for i,stamp in enumerate(stamps)]+[(10,2,15,b'ignored')])
        db.commit();db.close()
    rows=list(replay.records(tmp_path,['/wit/imu']))
    assert [r[0] for r in rows]==[10,20,30,40]
    assert all(r[1]=='/wit/imu' and r[2]==b'imu' for r in rows)


def test_pause_rate_and_finished_keep_node_alive(monkeypatch):
    if os.environ.get('ROS_DOMAIN_ID')!='213':pytest.skip('隔離DDS domain 213で実行する')
    stamp=1785025069000000000
    data=serialize_message(Imu())
    monkeypatch.setattr(replay,'records',lambda *_:iter([(stamp,'/wit/imu',data),(stamp+10**9,'/wit/imu',data)]))
    rclpy.init(args=[])
    node=replay.PerceptionBagPlayer(parameter_overrides=[Parameter('topics',value=['/wit/imu']),
        Parameter('start_paused',value=True),Parameter('startup_delay_sec',value=0.)])
    try:
        node.tick();assert not node.counts
        req=SetBool.Request();req.data=False;node.set_paused(req,SetBool.Response())
        node.tick();assert node.counts['/wit/imu']==1
        before=node.cursor
        assert node.set_parameters_atomically([Parameter('rate',value=2.)]).successful
        assert 0<=node.cursor-before<10**8
        assert not node.set_parameters_atomically([Parameter('rate',value=0.)]).successful
        req.data=True;node.set_paused(req,SetBool.Response());before=node.cursor
        node.last_wall-=1.;node.tick();assert node.cursor==before
        req.data=False;node.set_paused(req,SetBool.Response());node.cursor=stamp+2*10**9
        node.tick();assert node.finished and node.counts['/wit/imu']==2
        # 終端でpublisherを破棄しない。静的TFや最終画像を確認するGUIを残す。
        node.tick();assert node.finished and node.counts['/wit/imu']==2
    finally:node.destroy_node();rclpy.shutdown()


