"""実producer→GUI要求→MPPI→実車両nodeのROS接続を、USB非接続で検証する。

投影後の人工点群をproducerの実モデルへ渡す。カメラ・実ODriveは起動しない。
MotorControllerはimport前に置換し、USB探索・実モータ指令へ到達する経路を持たない。
"""
import importlib
import json
import os
import sys
import time
from pathlib import Path
from types import ModuleType,SimpleNamespace
import numpy as np
import pytest
import yaml

rclpy=pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
from pm_perception.planner_mapper_node import PlannerMapper
# GUIはplannerへ依存するため逆向きのtest_dependは循環になる。全stack試験時に別途installする。
RosBackend=pytest.importorskip('pm_gui.ros_backend',reason='全stack結合試験にはpm_guiが必要').RosBackend
from test_autonomous_pipeline import Rig


class MemoryMotor:
    """ODriveの代わりの記録先。通信ポート・実デバイスを所有しない。"""
    def __init__(self,*args,**kwargs):self.commands=[];self.active=kwargs.get('activate',True)
    def set_velocity(self,value):self.commands.append(float(value))
    def activate(self):self.active=True
    def deactivate(self):self.active=False


def test_real_producer_gui_mppi_and_vehicle_arbitration(tmp_path,monkeypatch):
    if os.environ.get('ROS_DOMAIN_ID')!='213':pytest.skip('ROS_DOMAIN_ID=213で隔離する')
    ws=Path(__file__).resolve().parents[3]
    # ハードウェア依存をimport前に遮断。実車両nodeの購読・許可・選択・変換は実行する。
    stub=ModuleType('pm_control.odrive_controller');stub.MotorController=MemoryMotor
    monkeypatch.setitem(sys.modules,'pm_control.odrive_controller',stub)
    previous=sys.modules.pop('pm_control.vehicle_interface_node',None)
    vehicle_module=importlib.import_module('pm_control.vehicle_interface_node')
    monkeypatch.setattr(vehicle_module,'MotorController',MemoryMotor)
    configs={
        'vehicle_interface_node':yaml.safe_load((ws/'src/pm_config/config/vehicle_control.yaml').read_text())['vehicle_interface_node']['ros__parameters'],
        'depth_elevation_mapper':yaml.safe_load((ws/'src/pm_config/config/planner_perception.yaml').read_text())['/**']['ros__parameters']}
    configs['vehicle_interface_node']['require_neutral_on_start']=True
    base_init=Node.__init__
    def init(node,name,*args,**kwargs):
        if name in configs:
            kwargs['parameter_overrides']=[Parameter(k,value=v) for k,v in dict(configs[name],use_sim_time=True).items()]
        base_init(node,name,*args,**kwargs)
    monkeypatch.setattr(Node,'__init__',init)
    rclpy.init(args=[]);r=None
    try:
        r=Rig(tmp_path)
        mission=yaml.safe_load((ws/'src/pm_planner/examples/demo_mission.yaml').read_text())
        r.until(lambda:r.upload.service_is_ready() and r.start.service_is_ready())
        r.send(mission)
        r.helper.destroy_service(r.permission_service)
        vehicle=vehicle_module.VehicleInterfaceNode();producer=PlannerMapper()
        gui_config=yaml.safe_load((ws/'src/pm_gui/config/gui.yaml').read_text())['operator_console']['ros__parameters']
        gui=RosBackend(gui_config)
        for node in (vehicle,producer,gui):
            r.nodes.append(node);r.executor.add_node(node)
            for timer in node.timers:timer.cancel()
        assert isinstance(vehicle.left_motor,MemoryMotor) and isinstance(vehicle.right_motor,MemoryMotor)
        assert vehicle.cmd_vel_topic=='/cmd_vel_autonomy' and not vehicle._autonomy_enabled
        def status(_):
            r.permission=vehicle._autonomy_enabled;vehicle.publish_vehicle_status()
        r.vehicle=SimpleNamespace(publish=status)
        static=StaticTransformBroadcaster(r.helper)
        transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='odom'
        transform.transform.rotation.w=1.;static.sendTransform(transform);r.pump(80)
        r.until(lambda:r.manager.tf_buffer.can_transform('map','odom',r.manager.get_clock().now())
                and r.preview.buffer.can_transform('map','odom',r.manager.get_clock().now()))
        # 実走行用と同じ中立インターロックを通す（Twistは0、宛先は隔離domainのメモリnode）。
        from geometry_msgs.msg import Twist
        r.joy.publish(Twist());r.pump(80);vehicle.command_selector()
        assert vehicle._drive_armed
        time.sleep(.31)
        for _ in range(4):r.feed()
        # セル内3点を持つ平坦な人工路面と、前方の高点。奥行き再計算や実センサではない。
        floor=np.array([(x+dx,y+dy,0.) for x in np.arange(-1.05,3.06,.1)
                        for y in np.arange(-1.05,1.06,.1)
                        for dx,dy in ((-.02,-.01),(0.,0.),(.02,.01))])
        scene=np.vstack((floor,[[1.45,.05,.2]]));enabled=True;last=-1
        original_feed=r.feed
        def feed(**kwargs):
            nonlocal last
            original_feed(**kwargs)
            stamp=r.manager.get_clock().now().nanoseconds
            if enabled and stamp-last>=int(round(producer.debug_period_sec*1e9)):
                producer.grid.model.update(scene,(0.,0.,.4),(0.,0.),stamp)
                producer.publish_debug(r.manager.get_clock().now().to_msg());last=stamp;r.pump(80)
            vehicle.command_selector();vehicle.publish_selected_cmd_vel();vehicle.publish_vehicle_status();r.pump(40)
        r.feed=feed
        r.until(gui._perception_mode_client.service_is_ready)
        assert gui.request_perception_mode('obstacle_only')
        r.until(lambda:r.manager.perception_mode=='obstacle_only' and not gui._perception_mode_pending)
        r.until(lambda:not r.manager.readiness())
        assert r.manager.terrain.sample(1.45,.05)==100
        assert r.manager.terrain.sample(0.,0.)==0
        assert gui.request_autonomy('start')
        r.until(lambda:r.manager.state=='RUNNING')
        for _ in range(5):r.feed()
        assert vehicle._autonomy_enabled and r.command[0]>0.
        assert any(abs(v)>0 for v in vehicle.left_motor.commands)
        assert any(abs(v)>0 for v in vehicle.right_motor.commands)
        assert vehicle.last_cmd_vel is not None and vehicle._selected_cmd_vel.linear.x>0.
        # plannerが新指令を出せなくても車両側の0.3秒期限でゼロになる。
        from rosgraph_msgs.msg import Clock
        from builtin_interfaces.msg import Time
        r.time+=.4;clock_ns=int(round(r.time*1e9))
        r.clock.publish(Clock(clock=Time(sec=clock_ns//1000000000,nanosec=clock_ns%1000000000)));r.pump(80)
        vehicle.command_selector()
        assert vehicle.left_motor.commands[-1]==0. and vehicle.right_motor.commands[-1]==0.
        r.feed()
        trajectory=next(t for t in reversed(r.trajectories) if t.get('valid'))['points']
        xy=np.array([(p['x'],p['y']) for p in trajectory]);before=np.vstack(((r.x,r.y),xy[:-1]))
        assert np.all(r.manager.terrain.segment(before[:,0],before[:,1],xy[:,0],xy[:,1])<100)
        # 実producerで足元にも高点を追加。plannerは危険を受けてゼロ出力へ戻る。
        scene=np.vstack((scene,[[.05,.05,.2]]))
        r.until(lambda:r.manager.state=='WAITING')
        r.feed();assert r.command==(0.,0.)
        assert vehicle.left_motor.commands[-1]==0. and vehicle.right_motor.commands[-1]==0.
        assert vehicle._autonomy_enabled
        # 既定の待機optionでは、健全な非検出mapへ戻れば許可を再要求せず続行する。
        scene=floor
        r.until(lambda:r.manager.state=='RUNNING' and not r.manager.readiness())
        assert vehicle._autonomy_enabled
        enabled=False
        r.until(lambda:r.manager.state=='DISABLED')
        assert '期限切れ' in r.manager.reason
        r.feed();assert vehicle.left_motor.commands[-1]==0. and vehicle.right_motor.commands[-1]==0.
        # Joyも車両側で許可を解除し、途絶だけでは自律へ復帰しない。
        vehicle._autonomy_enabled=True
        r.joy.publish(Twist());r.pump(80);vehicle.command_selector()
        assert not vehicle._autonomy_enabled and vehicle.last_cmd_vel is None
        if os.environ.get('PM_PLANNER_TEST_REPORT_DIR'):
            out=Path(os.environ['PM_PLANNER_TEST_REPORT_DIR']);out.mkdir(parents=True,exist_ok=True)
            (out/'producer_vehicle.json').write_text(json.dumps(dict(
                producer=type(producer).__name__,vehicle=type(vehicle).__name__,mode=r.manager.perception_mode,
                hazard_stamp=producer.grid.model.last_stamp,nonzero_autonomy=True,nonzero_memory_motor=True,
                max_autonomy_v_m_s=max(v for v,w in r.commands),
                max_abs_left_motor_rps=max(abs(v) for v in vehicle.left_motor.commands),
                max_abs_right_motor_rps=max(abs(v) for v in vehicle.right_motor.commands),
                global_tf_received=True,vehicle_command_timeout_zero=True,perception_timeout_zero=True,
                obstacle_rejected=True,final_command=r.command,hardware='MemoryMotor only'),indent=2))
    finally:
        if r is not None:r.close()
        rclpy.shutdown()
        sys.modules.pop('pm_control.vehicle_interface_node',None)
        if previous is not None:sys.modules['pm_control.vehicle_interface_node']=previous
