"""実機なしでROS callbackから表示までを検証する。Qtはoffscreenで実行する。"""
import os
import threading
import time
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5.QtWidgets import QApplication
from sensor_msgs.msg import Imu, MagneticField
from pm_gui.ros_backend import RosBackend
from pm_gui.panels import TelemetryPanel


def backend_stub():
    """subscription/serviceを起動せず、callbackの必要な状態だけ用意する。"""
    stub = SimpleNamespace(_lock=threading.RLock(), _wit_attitude=None,
                           _compass_config={}, topic_timeout=2, values={})
    stub._store = lambda key,value: stub.values.update({key:value})
    return stub


def test_callbacks_and_invalid():
    stub = backend_stub()
    mag = MagneticField()
    mag.header.frame_id = 'base_link'
    mag.magnetic_field.y = 1.0
    RosBackend._wit_mag_callback(stub,mag)
    assert stub.values['magnetic_heading']['bearing'] is None
    imu = Imu()
    imu.header.frame_id = 'base_link'
    imu.orientation.w = 1.0
    RosBackend._wit_imu_callback(stub,imu)
    RosBackend._wit_mag_callback(stub,mag)
    assert stub.values['magnetic_heading']['bearing'] == 90
    stub._wit_attitude = (0,0,'base_link',time.monotonic()-3)
    RosBackend._wit_mag_callback(stub,mag)
    assert stub.values['magnetic_heading']['bearing'] is None
    RosBackend._wit_imu_callback(stub,imu)
    mag.header.frame_id = 'different_frame'
    RosBackend._wit_mag_callback(stub,mag)
    assert 'frame' in stub.values['magnetic_heading']['error']


def test_panel_separates_odom_and_compass():
    app = QApplication.instance() or QApplication([])
    panel = TelemetryPanel({})
    snapshot = {'telemetry':{'odometry':{'fresh':True,'value':{'roll':0,'pitch':0,'yaw':0}}}}
    panel.refresh(snapshot,{})
    assert '待受' in panel.attitude.text()
    assert '方位 90' not in panel.attitude.text()
    snapshot['telemetry']['magnetic_heading'] = {'fresh':True,'value':{
        'bearing':90, 'calibrated':False, 'error':''}}
    panel.refresh(snapshot,{})
    assert '90.0° E' in panel.attitude.text()
    assert '未校正' in panel.attitude.text()
    panel.close()
