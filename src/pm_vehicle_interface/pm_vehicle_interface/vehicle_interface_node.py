import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from pm_msgs.msg import MotorState
from .odrive_controller import MotorController
import math
import sys


class VehicleInterfaceNode(Node):
    def __init__(self):
        super().__init__(
            "vehicle_interface_node",
        )  # register the node

        # パラメータ宣言（launchから上書き可）
        self.declare_parameter("wheel_radius", 0.1)
        self.declare_parameter("tread_width", 0.36)
        self.declare_parameter("gear_ratio", 10.0)
        self.declare_parameter("max_whl_rps", 4.0)

        self.declare_parameter("odrv_usb_port", "/dev/ttyACM0")
        self.declare_parameter("odrv_baud_rate", 115200)

        self.declare_parameter("mtr_axis_l", 0)
        self.declare_parameter("mtr_axis_r", 1)

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_joy_topic", "/cmd_vel_joy")
        self.declare_parameter("motor_state_topic", "/motor_state")
        self.declare_parameter("emergency_stop_topic", "/emergency_stop")

        self.declare_parameter("vel_ramp_rate", 15.0)
        self.declare_parameter("pos_gain", 30.0)
        self.declare_parameter("vel_gain", 0.225)
        self.declare_parameter("vel_integrator_gain", 0.75)
        self.declare_parameter("vel_integrator_limit", 2.0)

        self.declare_parameter("left_motor_sign", 1.0)
        self.declare_parameter("right_motor_sign", -1.0)

        # パラメータ取得
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.tread_width = self.get_parameter("tread_width").get_parameter_value().double_value
        self.gear_ratio = self.get_parameter("gear_ratio").get_parameter_value().double_value
        self.max_whl_rps = self.get_parameter("max_whl_rps").get_parameter_value().double_value

        self.odrv_usb_port = self.get_parameter("odrv_usb_port").get_parameter_value().string_value
        self.odrv_baud_rate = self.get_parameter("odrv_baud_rate").get_parameter_value().integer_value

        self.mtr_axis_l = self.get_parameter("mtr_axis_l").get_parameter_value().integer_value
        self.mtr_axis_r = self.get_parameter("mtr_axis_r").get_parameter_value().integer_value

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.cmd_vel_joy_topic = self.get_parameter("cmd_vel_joy_topic").get_parameter_value().string_value
        self.motor_state_topic = self.get_parameter("motor_state_topic").get_parameter_value().string_value
        self.emergency_stop_topic = self.get_parameter("emergency_stop_topic").get_parameter_value().string_value

        self.vel_ramp_rate = self.get_parameter("vel_ramp_rate").get_parameter_value().double_value
        self.pos_gain = self.get_parameter("pos_gain").get_parameter_value().double_value
        self.vel_gain = self.get_parameter("vel_gain").get_parameter_value().double_value
        self.vel_integrator_gain = self.get_parameter("vel_integrator_gain").get_parameter_value().double_value
        self.vel_integrator_limit = self.get_parameter("vel_integrator_limit").get_parameter_value().double_value

        self.left_motor_sign = self.get_parameter("left_motor_sign").get_parameter_value().double_value
        self.right_motor_sign = self.get_parameter("right_motor_sign").get_parameter_value().double_value

        # display the set parameters
        self.print_parameters()

        # connect to odrive
        self.left_motor = None
        self.right_motor = None
        self.odrive_connected = False
        self.reconnect_in_progress = False

        self.connect_odrive()
        self._reconnect_timer = self.create_timer(1.0, self.try_reconnect_odrive)

        # self.left_motor.get_velocity()

        # param definition related to subscriptions
        self.last_cmd_vel = None
        self.last_cmd_vel_time = None
        self.last_cmd_vel_joy = None
        self.last_cmd_vel_joy_time = None
        # subscriber config
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.create_subscription(
            Twist, self.cmd_vel_joy_topic, self.cmd_vel_callback_joy, 10
        )
        self.create_subscription(
            Bool, self.emergency_stop_topic, self.emergency_stop_callback, 10
        )

        # ------------------------
        # other params
        # ------------------------
        # self.current_vel_left = 0.0
        # self.last_vel_left = 0.0
        # self.current_vel_right = 0.0
        # self.last_vel_right = 0.0
        # self.accumerated_ver_err_left = 0.0
        # self.accumerated_ver_err_right = 0.0

        # タイマーを定義、設定時間（sec）ごとに関数を呼び出す（遠隔操縦指令の受領関数と、モータ制御情報の発信関数）
        self._timer = self.create_timer(0.05, self.command_selector)
        self._motor_state_timer = self.create_timer(0.05, self.publish_motor_state)

        # publihser config
        self.motor_state_pub = self.create_publisher(
            MotorState, self.motor_state_topic, 10
        )
        self.sim_cmd_vel_pub = self.create_publisher(Twist, "/sim_cmd_vel", 10)

    def connect_odrive(self):
        """Connect/Re-Connect to ODrive and initialize both motors."""
        try:
            self.get_logger().info("connecting to ODrive...")
            self.left_motor = MotorController(
                self.mtr_axis_l,
                vel_ramp_rate=self.vel_ramp_rate,
                pos_gain=self.pos_gain,
                vel_gain=self.vel_gain,
                vel_integrator_gain=self.vel_integrator_gain,
                vel_integrator_limit=self.vel_integrator_limit,
            )

            self.right_motor = MotorController(
                self.mtr_axis_r,
                vel_ramp_rate=self.vel_ramp_rate,
                pos_gain=self.pos_gain,
                vel_gain=self.vel_gain,
                vel_integrator_gain=self.vel_integrator_gain,
                vel_integrator_limit=self.vel_integrator_limit,
            )

            self.left_cmd_rps = 0.0  # モータ指令値をpublishするために値を保存しておく変数（左）
            self.right_cmd_rps = 0.0  # モータ指令値をpublishするために値を保存しておく変数（右）
            self.odrive_connected = True
            self.reconnect_in_progress = False

            self.get_logger().info("ODrive connected and initialized!")

        except Exception as e:
            self.left_motor = None
            self.right_motor = None
            self.odrive_connected = False
            self.reconnect_in_progress = False
            self.get_logger().warn(f"ODrive connection failed: {e}")

    def cmd_vel_callback(self, msg):
        """callback function when /cmd_vel from autnomous driving software has been recieved"""
        self.last_cmd_vel = msg  # keep /cmd_vel_msg
        self.last_cmd_vel_time = (
            self.get_clock().now()
        )  # log the time when the msg received

    def cmd_vel_callback_joy(self, msg):
        """callback function when /cmd_vel from autnomous driving software has been recieved"""
        self.last_cmd_vel_joy = msg  # keep /cmd_vel_time_msg
        self.last_cmd_vel_joy_time = (
            self.get_clock().now()
        )  # log the time when the msg received

    def command_selector(self):
        """check which command should be prioritized, from gamepad or autonomous driving software"""
        try:
            now = self.get_clock().now()
            cmd = None

            # prioritize /cmd_vel_joy from gamepad
            if self.last_cmd_vel_joy is not None:
                # check the command's newness
                if (now - self.last_cmd_vel_joy_time).nanoseconds < 0.3 * 1e9:
                    cmd = self.last_cmd_vel_joy

            # use /cmd_vel when no joy cmd received
            if cmd is None and self.last_cmd_vel is not None:
                # check the command's newness
                if (now - self.last_cmd_vel_time).nanoseconds < 0.3 * 1e9:
                    cmd = self.last_cmd_vel

            # control motor:
            self.motor_control(cmd)

        except Exception as e:
            self.get_logger().error(f"Exception in command_selector: {e}")

    def motor_control(self, cmd):
        """convert command to motor speed and send to ODrive"""
        if cmd is not None:
            lin_x = cmd.linear.x  # velocity (forward/backward)
            ang_z = cmd.angular.z  # velocity (turning)
            wheel_perimeter = self.wheel_radius * 2 * math.pi  # wheel perimeter

            # compute rps of L/R wheels
            v_left = (lin_x - ang_z * self.tread_width / 2) / wheel_perimeter
            v_right = (lin_x + ang_z * self.tread_width / 2) / wheel_perimeter

            # cap by max speed
            v_left = max(min(v_left, self.max_whl_rps), -self.max_whl_rps)
            v_right = max(min(v_right, self.max_whl_rps), -self.max_whl_rps)

            # convert to motor rps
            mtr_left_rps = v_left * self.gear_ratio
            mtr_right_rps = v_right * self.gear_ratio

            # publishe the equivalent Twist command for simulation
            self.sim_cmd_vel_pub.publish(cmd)

        else:
            mtr_left_rps = 0.0
            mtr_right_rps = 0.0

            # if the command is None, publihs it for simulation
            zero_cmd = Twist()
            self.sim_cmd_vel_pub.publish(zero_cmd)

        if not self.odrive_connected:
            self.left_cmd_rps = 0.0
            self.right_cmd_rps = 0.0
            return

        try:
            # send command to ODrive (右モータの速度は反転)
            self.left_motor.set_velocity(self.left_motor_sign * mtr_left_rps)
            self.right_motor.set_velocity(self.right_motor_sign * mtr_right_rps)

            # keep command values in vehicle coordinate convention
            self.left_cmd_rps = mtr_left_rps
            self.right_cmd_rps = mtr_right_rps

        except Exception as e:
            self.mark_odrive_disconnected(e)

        # # get current and past motor velocity with low pass filter
        # self.last_vel_left = self.current_vel_left
        # self.last_vel_right = self.current_vel_right
        # self.current_vel_left = (
        #     0.2 * self.left_motor.get_velocity() + 0.8 * self.last_vel_left
        # )
        # self.current_vel_right = (
        #     0.2 * self.right_motor.get_velocity() + 0.8 * self.last_vel_right
        # )

        # # velocity feedback torque control
        # vel_err_left = mtr_left_rps - self.current_vel_left             # for P control
        # vel_err_right = mtr_right_rps - self.current_vel_right          # for P control
        # delta_vel_left = self.current_vel_left - self.last_vel_left     # for D control
        # delta_vel_right = self.current_vel_right - self.last_vel_right  # for D control
        # self.accumerated_ver_err_left = max(
        #     self.accumerated_ver_err_left + self.current_vel_left - mtr_left_rps, 5
        # )                                                               # for I control
        # self.accumerated_ver_err_right = max(
        #     self.accumerated_ver_err_right + self.current_vel_right - mtr_right_rps, 5
        # )                                                               # for I control
        # self.left_motor.velfb_torque_control(
        #     vel_err_left, delta_vel_left, self.accumerated_ver_err_left
        # )
        # self.right_motor.velfb_torque_control(
        #     vel_err_right, delta_vel_right, self.accumerated_ver_err_right
        # )

    def publish_motor_state(self):
        """モータ制御情報を取得しpublishする関数"""
        if not self.odrive_connected:
            return

        try:
            msg = MotorState()
            msg.stamp = self.get_clock().now().to_msg()

            # 速度指令値
            msg.left_cmd_rps = float(self.left_cmd_rps)
            msg.right_cmd_rps = float(self.right_cmd_rps)

            # エンコーダ値（1回転で1増えるturns単位）
            msg.left_pos_turns = float(
                self.left_motor_sign * self.left_motor.get_position()
            )
            msg.right_pos_turns = float(
                self.right_motor_sign * self.right_motor.get_position()
            )

            # モータ回転速度（実績）
            msg.left_vel_rps = float(
                self.left_motor_sign * self.left_motor.get_velocity()
            )
            msg.right_vel_rps = float(
                self.right_motor_sign * self.right_motor.get_velocity()
            )

            # q軸電流 [A]　実績
            # （符号も車体座標系に合わせるなら motor_sign を掛けるべき？）
            msg.left_iq_measured_a = float(self.left_motor.get_iq_measured())
            msg.right_iq_measured_a = float(self.right_motor_sign * self.right_motor.get_iq_measured())
            # q軸電流 [A]　指令値
            msg.left_iq_setpoint_a = float(self.left_motor_sign * self.left_motor.get_iq_setpoint())
            msg.right_iq_setpoint_a = float(self.right_motor_sign * self.right_motor.get_iq_setpoint())

            # 電源電圧
            msg.vbus_voltage = float(self.left_motor.get_vbus_voltage())

            # publish
            self.motor_state_pub.publish(msg)

            pass

        except Exception as e:
            self.mark_odrive_disconnected(e)

    def emergency_stop_callback(self, msg):
        """emefgency stop: stop motors immediately"""
        if msg.data:
            self.left_cmd_rps = 0.0
            self.right_cmd_rps = 0.0

            if self.odrive_connected:
                try:
                    self.left_motor.set_idle()
                    self.right_motor.set_idle()
                except Exception as e:
                    self.mark_odrive_disconnected(e)

            self.get_logger().warn("Emergency STOP activated!")

    def stop_motors(self):
        """in case of emergency cases: motor stop"""
        self.left_motor.set_idle()
        self.right_motor.set_idle()
        self.get_logger().warn("No !")

    def print_parameters(self):
        header = f"{'Parameter':<20} {'Value':<20}"
        separator = "-" * 42
        lines = [header, separator]
        for key, value in [
            ("wheel_radius", self.wheel_radius),
            ("tread_width", self.tread_width),
            ("gear_ratio", self.gear_ratio),
            ("max_whl_rps", self.max_whl_rps),
            ("odrv_usb_port", self.odrv_usb_port),
            ("odrv_baud_rate", self.odrv_baud_rate),
            ("mtr_axis_l", self.mtr_axis_l),
            ("mtr_axis_r", self.mtr_axis_r),
            ("cmd_vel_topic", self.cmd_vel_topic),
            ("cmd_vel_joy_topic", self.cmd_vel_joy_topic),
            ("motor_state_topic", self.motor_state_topic),
            ("emergency_stop_topic", self.emergency_stop_topic),
            ("vel_ramp_rate", self.vel_ramp_rate),
            ("pos_gain", self.pos_gain),
            ("vel_gain", self.vel_gain),
            ("vel_integrator_gain", self.vel_integrator_gain),
            ("vel_integrator_limit", self.vel_integrator_limit),
        ]:
            lines.append(f"{key:<20} {str(value):<20}")
        self.get_logger().info("\n".join(lines))

    def try_reconnect_odrive(self):
        """Try reconnecting to ODrive when disconnected."""
        if self.odrive_connected:
            return

        if self.reconnect_in_progress:
            return

        self.reconnect_in_progress = True
        self.get_logger().warn("ODrive disconnected. Trying to reconnect...")
        self.connect_odrive()

    def mark_odrive_disconnected(self, reason):
        """Mark ODrive as disconnected."""
        if self.odrive_connected:
            self.get_logger().error(f"ODrive disconnected: {reason}")

        self.odrive_connected = False
        self.left_motor = None
        self.right_motor = None
        self.left_cmd_rps = 0.0
        self.right_cmd_rps = 0.0


def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2
    node = VehicleInterfaceNode()  # Create node instance

    try:
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down gracefully...")  # Log clean shutdown
    finally:
        if rclpy.ok():  # Prevent multiple shutdown calls
            node.destroy_node()  # Destroy node properly
            rclpy.shutdown()  # Shutdown ROS2 cleanly
        sys.exit(0)  # Exit without error


if __name__ == "__main__":
    main()
