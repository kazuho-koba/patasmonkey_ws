"""実センサ・ODriveを接続せず、preview・設定変更・IDLE専用取得を検証する。"""
import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry, Path as RosPath
from pm_msgs.msg import TerrainAssessment
from pm_control.motor_state_observer import ensure_idle
from pm_perception.planner_mapper_node import PlannerMapper
from pm_planner.perception_preview import PerceptionPreview


def test_idle_observer_never_sets_velocity():
    """許可した書込みはIDLE要求だけ。疑似deviceに速度API自体を用意しない。"""
    class Axis:
        __slots__=('current_state','requested_state')
        def __init__(self,state):self.current_state=state;self.requested_state=None
    device=SimpleNamespace(axis0=Axis(8),axis1=Axis(1))
    ensure_idle(device,1)
    assert device.axis0.requested_state==1 and device.axis1.requested_state is None


def test_lab_launch_has_opt_in_drive():
    """launch記述の生成だけを行う。外部プロセスは実行しない。"""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, GroupAction
    path=Path(__file__).resolve().parents[1]/'launch/perception_lab.launch.py'
    spec=importlib.util.spec_from_file_location('perception_lab_launch',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    context=LaunchContext()
    entities=module.generate_launch_description().entities
    for action in entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    assert context.launch_configurations['drive']=='false'
    assert context.launch_configurations['params_file'].endswith('pm_config/config/planner_perception.yaml')
    outer=next(action for action in entities if isinstance(action,GroupAction))
    scopes=[action for action in outer.get_sub_entities() if isinstance(action,GroupAction)]
    assert len(scopes)==2
    # 遅延起動するCoreのuse_joy等が、Driveのinclude引数と混ざらないことを確認。
    context.launch_configurations['use_joy']='outer'
    for group in scopes:
        actions=group.execute(context)
        actions[0].execute(context);actions[1].execute(context)
        context.launch_configurations['use_joy']='inner'
        actions[-2].execute(context);actions[-1].execute(context)
        assert context.launch_configurations['use_joy']=='outer'


@pytest.fixture
def ros():
    if os.environ.get('ROS_DOMAIN_ID')!='213':pytest.skip('隔離DDS domain 213で実行する')
    rclpy.init(args=[])
    yield
    rclpy.shutdown()


def test_producer_live_parameter_validation_and_reset(ros,monkeypatch):
    """不正変更で既存mapを壊さず、正しい変更で空mapへ切替える。"""
    init=Node.__init__
    def override(node,name,*args,**kwargs):
        if name=='depth_elevation_mapper':
            kwargs['parameter_overrides']=[Parameter('map_size_x',value=8.),
                Parameter('map_size_y',value=8.),Parameter('resolution',value=.1)]
        init(node,name,*args,**kwargs)
    monkeypatch.setattr(Node,'__init__',override)
    node=PlannerMapper()
    try:
        before=node.grid
        result=node.set_parameters_atomically([Parameter('max_depth',value=.2)])
        assert not result.successful and node.grid is before
        result=node.set_parameters_atomically([Parameter('max_depth',value=7.),Parameter('resolution',value=.2)])
        assert result.successful and node.grid is not before
        assert node.max_depth==7. and node.grid.width==40 and not node.grid.model.cells
        assert not node.set_parameters_atomically([Parameter('depth_subscription_queue_depth',value=20)]).successful
    finally:node.destroy_node()


def test_preview_real_cost_input_and_unknown_blocks_path(ros,monkeypatch):
    """認識message→cost選択→実MPPIのPathを確認。速度topicのpublisherは存在しない。"""
    from pm_planner import perception_preview as preview_module
    original=preview_module.TerrainCost;environments=[]
    def record_environment(*args,**kwargs):
        env=original(*args,**kwargs);environments.append(env);return env
    monkeypatch.setattr(preview_module,'TerrainCost',record_environment)
    node=PerceptionPreview(parameter_overrides=[Parameter('require_heading_ready',value=False)])
    helper=Node('perception_preview_test');paths=[];maps=[];profiles=[];markers=[]
    import json
    from std_msgs.msg import String
    from visualization_msgs.msg import MarkerArray, Marker
    helper.create_subscription(String,'/perception_preview/speed_profile',lambda m:profiles.append(json.loads(m.data)),10)
    helper.create_subscription(MarkerArray,'/perception_preview/speed_markers',markers.append,10)
    from nav_msgs.msg import OccupancyGrid
    helper.create_subscription(RosPath,'/perception_preview/local_path',paths.append,10)
    helper.create_subscription(OccupancyGrid,'/perception_preview/selected_costmap',maps.append,10)
    import time
    try:
        # 通常plannerの正本と、表示専用nodeの速度・内部予測窓が一致する。
        import yaml
        from ament_index_python.packages import get_package_share_directory
        shared=yaml.safe_load((Path(get_package_share_directory('pm_config'))/'config/planner.yaml').read_text())['/**']['ros__parameters']
        assert node.get_parameter('target_speed').value==shared['target_speed']
        assert node.get_parameter('prediction_steps').value==shared['steps']
        assert node.get_parameter('prediction_dt').value==1./shared['controller_rate_hz']
        assert node.get_parameter('footprint_radius_m').value==shared['footprint_radius_m']
        assert node.get_parameter('footprint_margin_m').value==shared['footprint_margin_m']
        # discoveryを待つ間も外部hardwareは起動しない。
        until=time.monotonic()+.5
        while time.monotonic()<until:rclpy.spin_once(helper,timeout_sec=.02)
        pose=Odometry();pose.header.frame_id='odom';pose.child_frame_id='base_link'
        pose.pose.pose.orientation.w=1.
        msg=TerrainAssessment();msg.schema_version=1;msg.header.frame_id='odom'
        msg.info.resolution=.1;msg.info.width=80;msg.info.height=80
        msg.info.origin.position.x=-4.;msg.info.origin.position.y=-4.;msg.info.origin.orientation.w=1.
        msg.obstacle_cost=[0]*6400;msg.terrain_cost=[100]*6400
        def tick():
            stamp=node.get_clock().now().to_msg();pose.header.stamp=stamp;msg.header.stamp=stamp
            node.on_pose(pose);node.on_map(msg);node.tick()
            until=time.monotonic()+.15
            while time.monotonic()<until:rclpy.spin_once(helper,timeout_sec=.01)
        tick()
        assert paths and paths[-1].poses and maps[-1].data[0]==0
        assert profiles[-1]['valid']
        assert len(profiles[-1]['speed_mps'])==shared['steps']
        assert profiles[-1]['cap_mps']==pytest.approx([shared['target_speed']]*shared['steps'])
        assert profiles[-1]['actual_mps']==0.
        assert markers[-1].markers[0].action==Marker.DELETEALL
        assert len(markers[-1].markers[1].points)==shared['steps']+1
        # 上限の照会位置は各区間の開始点。終点で照会すると近接時の値がずれる。
        queried=[]
        def risk(x,y,preferred):
            queried.append((np.asarray(x).copy(),np.asarray(y).copy()))
            return np.array([0.,1.])
        synthetic=dict(trajectory=np.array([[.1,0.,0.],[.2,0.,0.]]),
                       trajectory_controls=np.array([[.3,0.],[.2,0.]]))
        node.publish_speed(synthetic,SimpleNamespace(clearance_risk=risk))
        until=time.monotonic()+.1
        while time.monotonic()<until:rclpy.spin_once(helper,timeout_sec=.01)
        assert queried[0][0]==pytest.approx([0.,.1])
        assert profiles[-1]['distance_m']==pytest.approx([0.,.1])
        assert profiles[-1]['speed_mps']==pytest.approx([.3,.2])
        assert profiles[-1]['cap_mps']==pytest.approx([shared['target_speed'],min(shared['clearance_min_speed_mps'],shared['target_speed'])])
        assert environments[-1].inflation_radius==pytest.approx(shared['footprint_radius_m']+shared['footprint_margin_m'])
        # 半径と余裕は独立に変更できる。余裕0でもセル幅が再加算されない。
        changed=node.set_parameters_atomically([Parameter('footprint_radius_m',value=.2),
                                               Parameter('footprint_margin_m',value=0.)])
        assert changed.successful
        tick();assert environments[-1].inflation_radius==pytest.approx(.2)
        assert not node.set_parameters_atomically([Parameter('footprint_margin_m',value=-.1)]).successful
        assert node.get_parameter('footprint_margin_m').value==0.
        pose.pose.pose.position.x=.5;tick()
        assert node.reference[0,0]==0.  # online既定は最初の直線を維持する
        node.set_parameters_atomically([Parameter('reference_mode',value='current_pose')]);tick()
        assert node.reference[0,0]==.5  # offline指定はその地点から直進を仮定する
        result=node.set_parameters_atomically([Parameter('mode',value='full_terrain')]);assert result.successful
        tick();assert not paths[-1].poses and maps[-1].data[0]==100
        assert profiles[-1]=={'valid':False}
        assert len(markers[-1].markers)==1 and markers[-1].markers[0].action==Marker.DELETEALL
        node.set_parameters_atomically([Parameter('mode',value='obstacle_only')])
        msg.obstacle_cost=[-1]*6400;tick()
        assert not paths[-1].poses and maps[-1].data[0]==-1
        pubs=[topic for topic,_ in node.get_publisher_names_and_types_by_node(node.get_name(),'/')]
        assert not any('cmd_vel' in topic or 'motor_cmd' in topic for topic in pubs)
    finally:helper.destroy_node();node.destroy_node()


def test_gui_reads_effective_parameters_and_applies_atomic_change(ros,monkeypatch,tmp_path):
    """実QtとROS serviceで読込・変更を確認する。camera・motorは起動しない。"""
    from PyQt5.QtWidgets import QApplication
    from pm_gui.perception_lab import LabWindow
    app=QApplication.instance() or QApplication([])
    init=Node.__init__
    def override(node,name,*args,**kwargs):
        if name=='depth_elevation_mapper':
            kwargs['parameter_overrides']=[Parameter('map_size_x',value=8.),
                Parameter('map_size_y',value=8.),Parameter('resolution',value=.1),
                Parameter('max_depth',value=7.)]
        init(node,name,*args,**kwargs)
    monkeypatch.setattr(Node,'__init__',override)
    producer=PlannerMapper();preview=PerceptionPreview();panel=Node('lab_panel_test')
    panel.declare_parameter('drive_enabled',False);window=LabWindow(panel)
    import time
    def until(check):
        deadline=time.monotonic()+4.
        while not check() and time.monotonic()<deadline:
            rclpy.spin_once(producer,timeout_sec=.001)
            rclpy.spin_once(preview,timeout_sec=.001)
            window.poll();app.processEvents()
        assert check()
    try:
        window.show();until(lambda:window.loaded)
        assert window.widgets['max_depth'].value()==7.
        assert window.radius.value()==preview.get_parameter('footprint_radius_m').value
        assert window.margin.value()==preview.get_parameter('footprint_margin_m').value
        assert window.clearance['clearance_distance_m'].value()==preview.get_parameter('clearance_distance_m').value
        window.widgets['hazard_obstacle_height_limit'].setValue(.12)
        window.apply()
        until(lambda:window.pending is None and '認識・preview設定を反映' in window.result.text())
        assert producer.hazard_obstacle_height_limit==.12
        before=producer.grid
        window.radius.setValue(.25);window.margin.setValue(0.)
        window.clearance['clearance_distance_m'].setValue(.8)
        window.clearance['clearance_cost_weight'].setValue(4.)
        window.clearance['clearance_min_speed_mps'].setValue(.15)
        window.mode.setCurrentIndex(1);window.apply_preview()
        until(lambda:window.pending is None and preview.get_parameter('mode').value=='full_terrain')
        assert preview.get_parameter('footprint_radius_m').value==.25
        assert preview.get_parameter('footprint_margin_m').value==0.
        assert preview.get_parameter('clearance_distance_m').value==.8
        assert preview.get_parameter('clearance_cost_weight').value==4.
        assert preview.get_parameter('clearance_min_speed_mps').value==.15
        assert '0.250 m' in window.inflation.text()
        assert producer.grid is before
        from std_msgs.msg import String
        import json
        window.on_speed(String(data=json.dumps(dict(valid=True,distance_m=[0.,.2],
            speed_mps=[.3,.2],cap_mps=[1.,.2],target_mps=1.,actual_mps=.4,color_max_mps=1.))))
        app.processEvents()
        assert '0.30 → 0.20' in window.speed_summary.text()
        window.grab().save(str(tmp_path/'perception_lab_panel.png'))
        window.on_speed(String(data='{"valid": false}'))
        assert not window.speed_plot.data['valid']
    finally:
        window.timer.stop();window.close();panel.destroy_node();preview.destroy_node();producer.destroy_node()
