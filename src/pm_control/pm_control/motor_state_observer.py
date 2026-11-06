"""非駆動確認用ODrive状態取得。速度入力・駆動許可serviceは一切持たない。

MotorControllerの初期化を使わず、速度・gain・校正設定を書き換えない。
唯一の書込みは無励磁IDLE要求。エンコーダを読んで手押し移動のwheel odometryへ渡す。
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from pm_msgs.msg import MotorState


def ensure_idle(device, idle):
    """外部要因でclosed-loopになった場合もIDLEへ戻す。速度指令は送信しない。"""
    for axis in (device.axis0,device.axis1):
        if axis.current_state!=idle:axis.requested_state=idle


class MotorStateObserver(Node):
    def __init__(self):
        super().__init__('motor_state_observer')
        for name,value in (('left_motor_sign',1.),('right_motor_sign',-1.),
                           ('mtr_axis_l',0),('mtr_axis_r',1)):
            self.declare_parameter(name,value)
        self.device=None;self.next_connect=0.;self.last_warning=-10.;self.voltage=0.;self.last_voltage=-10.
        self.publisher=self.create_publisher(MotorState,'/motor_state',10)
        self.create_timer(1./30.,self.tick)

    def tick(self):
        # USB探索はこの専用processだけで行う。失敗時に架空のゼロ位置をpublishしない。
        now=self.get_clock().now().nanoseconds/1e9
        try:
            if self.device is None:
                if now<self.next_connect:return
                self.next_connect=now+2.
                import odrive
                self.device=odrive.find_any(timeout=1)
                if self.device is None:raise RuntimeError('ODriveが見つかりません')
            from odrive.enums import AXIS_STATE_IDLE
            ensure_idle(self.device,AXIS_STATE_IDLE)
            left=getattr(self.device,'axis'+str(self.get_parameter('mtr_axis_l').value))
            right=getattr(self.device,'axis'+str(self.get_parameter('mtr_axis_r').value))
            ls=self.get_parameter('left_motor_sign').value;rs=self.get_parameter('right_motor_sign').value
            start=self.get_clock().now().nanoseconds
            positions=(float(left.encoder.pos_estimate)*ls,float(right.encoder.pos_estimate)*rs)
            end=self.get_clock().now().nanoseconds
            velocities=(float(left.encoder.vel_estimate)*ls,float(right.encoder.vel_estimate)*rs)
            if not all(math.isfinite(v) for v in positions+velocities):raise ValueError('encoderが非有限')
            if now-self.last_voltage>1.:
                self.voltage=float(self.device.vbus_voltage);self.last_voltage=now
            msg=MotorState();msg.stamp=Time(nanoseconds=(start+end)//2).to_msg()
            msg.left_pos_turns,msg.right_pos_turns=positions
            msg.left_vel_rps,msg.right_vel_rps=velocities
            # cmd欄は非駆動の0。input_velへの書込みを意味しない。
            msg.left_cmd_rps=0.;msg.right_cmd_rps=0.;msg.vbus_voltage=self.voltage
            msg.ibus_a=float(self.device.ibus);self.publisher.publish(msg)
        except Exception as exc:
            self.device=None
            if now-self.last_warning>3.:
                self.get_logger().warn('非駆動ODrive状態取得: '+str(exc));self.last_warning=now


def main(args=None):
    rclpy.init(args=args);node=MotorStateObserver()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
