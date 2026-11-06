"""offline launchのgraphを生成し、hardware・旧TFが混入しないことを確認する。"""
import importlib.util
from pathlib import Path
import yaml
import pytest


def test_offline_graph_is_sim_time_and_has_no_hardware(tmp_path,monkeypatch):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    path=Path(__file__).resolve().parents[1]/'launch/perception_bag_lab.launch.py'
    spec=importlib.util.spec_from_file_location('bag_lab_launch',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    topics=('/wheel/odometry','/wit/imu','/oak/depth/image_raw','/ov_msckf/odomimu','/tf_static')
    metadata={'rosbag2_bagfile_information':{'topics_with_message_count':[
        {'topic_metadata':{'name':name,'type':'test'},'message_count':1} for name in topics]}}
    (tmp_path/'metadata.yaml').write_text(yaml.safe_dump(metadata))
    context=LaunchContext();context.launch_configurations['bag']=str(tmp_path)
    for action in module.generate_launch_description().entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    with pytest.raises(RuntimeError,match='CameraInfo'):module.setup(context)
    context.launch_configurations['old_bag']='true'
    captured=[];original=module.Node
    def node(**kwargs):captured.append(kwargs);return original(**kwargs)
    monkeypatch.setattr(module,'Node',node)
    module.setup(context)
    assert not {n['package'] for n in captured}&{'depthai_driver','pm_control','hwt905_rs485_driver','joy','teleop_twist_joy','ov_msckf'}
    assert not any(n['executable']=='execution_manager' for n in captured)
    player=next(n for n in captured if n['executable']=='perception_bag_player')
    assert '/tf' not in player['parameters'][0]['topics']
    assert '/ov_msckf/odomimu' in player['parameters'][0]['topics']
    for n in captured:
        if n is player:continue
        assert any(p.get('use_sim_time') is True for p in n.get('parameters',[]) if isinstance(p,dict))


def test_offline_gui_pause_and_rate_controls(monkeypatch):
    # GUIとの結合試験は双方に依存するbringup側に置き、evaluation→GUIの依存循環を作らない。
    import os
    import time
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu
    from pm_evaluation import perception_bag_player as replay
    if os.environ.get('ROS_DOMAIN_ID')!='213':pytest.skip('隔離DDS domain 213で実行する')
    from PyQt5.QtWidgets import QApplication
    from rclpy.node import Node
    from pm_gui.perception_lab import LabWindow
    data=serialize_message(Imu());stamp=1785025069000000000
    monkeypatch.setattr(replay,'records',lambda *_:iter([(stamp,'/wit/imu',data),(stamp+10**12,'/wit/imu',data)]))
    rclpy.init(args=[]);app=QApplication.instance() or QApplication([])
    player=replay.PerceptionBagPlayer(parameter_overrides=[Parameter('topics',value=['/wit/imu']),
        Parameter('start_paused',value=True),Parameter('startup_delay_sec',value=0.)])
    panel=Node('offline_panel_test');panel.declare_parameter('drive_enabled',False);panel.declare_parameter('offline_mode',True)
    window=LabWindow(panel)
    def until(check):
        end=time.monotonic()+4.
        while not check() and time.monotonic()<end:
            rclpy.spin_once(player,timeout_sec=.002);window.poll();app.processEvents()
        assert check()
    try:
        until(lambda:window.pause_client.service_is_ready() and window.rate_client.service_is_ready())
        window.pause(False);until(lambda:not player.paused)
        window.pause(True);until(lambda:player.paused)
        window.send(window.rate_client,{'rate':.5},lambda:None);until(lambda:player.rate==.5)
    finally:
        window.timer.stop();window.close();panel.destroy_node();player.destroy_node();rclpy.shutdown()
