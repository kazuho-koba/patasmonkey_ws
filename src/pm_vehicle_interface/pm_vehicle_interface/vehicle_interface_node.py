import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from pm_msgs.msg import MotorState
from rclpy.time import Time
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

        # ------------------------
        # 車両旋回制約
        # ------------------------
        self.declare_parameter("min_turn_radius", 0.75)  # [m]
        self.declare_parameter("max_yaw_rate", 0.5)  # [rad/s]
        self.declare_parameter("max_yaw_accel", 0.5)  # [rad/s^2]

        # joy入力の角度解釈
        self.declare_parameter("joy_max_turn_angle_deg", 45.0)  # [deg]
        self.declare_parameter("joy_side_stop_angle_deg", 80.0)  # [deg]

        # パラメータ取得
        self.wheel_radius = (
            self.get_parameter("wheel_radius").get_parameter_value().double_value
        )
        self.tread_width = (
            self.get_parameter("tread_width").get_parameter_value().double_value
        )
        self.gear_ratio = (
            self.get_parameter("gear_ratio").get_parameter_value().double_value
        )
        self.max_whl_rps = (
            self.get_parameter("max_whl_rps").get_parameter_value().double_value
        )

        self.odrv_usb_port = (
            self.get_parameter("odrv_usb_port").get_parameter_value().string_value
        )
        self.odrv_baud_rate = (
            self.get_parameter("odrv_baud_rate").get_parameter_value().integer_value
        )

        self.mtr_axis_l = (
            self.get_parameter("mtr_axis_l").get_parameter_value().integer_value
        )
        self.mtr_axis_r = (
            self.get_parameter("mtr_axis_r").get_parameter_value().integer_value
        )

        self.cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self.cmd_vel_joy_topic = (
            self.get_parameter("cmd_vel_joy_topic").get_parameter_value().string_value
        )
        self.motor_state_topic = (
            self.get_parameter("motor_state_topic").get_parameter_value().string_value
        )
        self.emergency_stop_topic = (
            self.get_parameter("emergency_stop_topic")
            .get_parameter_value()
            .string_value
        )

        self.vel_ramp_rate = (
            self.get_parameter("vel_ramp_rate").get_parameter_value().double_value
        )
        self.pos_gain = (
            self.get_parameter("pos_gain").get_parameter_value().double_value
        )
        self.vel_gain = (
            self.get_parameter("vel_gain").get_parameter_value().double_value
        )
        self.vel_integrator_gain = (
            self.get_parameter("vel_integrator_gain").get_parameter_value().double_value
        )
        self.vel_integrator_limit = (
            self.get_parameter("vel_integrator_limit")
            .get_parameter_value()
            .double_value
        )

        self.left_motor_sign = (
            self.get_parameter("left_motor_sign").get_parameter_value().double_value
        )
        self.right_motor_sign = (
            self.get_parameter("right_motor_sign").get_parameter_value().double_value
        )

        self.min_turn_radius = (
            self.get_parameter("min_turn_radius").get_parameter_value().double_value
        )
        self.max_yaw_rate = (
            self.get_parameter("max_yaw_rate").get_parameter_value().double_value
        )
        self.max_yaw_accel = (
            self.get_parameter("max_yaw_accel").get_parameter_value().double_value
        )

        self.joy_max_turn_angle_deg = (
            self.get_parameter("joy_max_turn_angle_deg")
            .get_parameter_value()
            .double_value
        )
        self.joy_side_stop_angle_deg = (
            self.get_parameter("joy_side_stop_angle_deg")
            .get_parameter_value()
            .double_value
        )

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
        # yaw rate ramp limiter 用
        self.prev_yaw_rate_cmd = 0.0
        self.prev_yaw_rate_time = None

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

        # タイマーを定義
        # - 速度指令のODrive反映: 25 Hz
        # - MotorStateのpublish: 30 Hz
        self._timer = self.create_timer(0.04, self.command_selector)
        self._motor_state_timer = self.create_timer(1.0 / 30.0, self.publish_motor_state)

        # vbus_voltageは変化が遅いため、ODriveからは1 Hzでのみ再取得する。
        # MotorState自体は30 Hzでpublishし、直近のキャッシュ値を載せる。
        self._vbus_voltage = 0.0
        self._last_vbus_update_time = None
        self._vbus_update_period_sec = 1.0

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

            # 再接続後の最初のMotorState publishでvbusを即時再取得する
            self._last_vbus_update_time = None

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
        """
        callback function when /cmd_vel_joy from gamepad has been received.
        joy_teleop由来のTwistを、車両制約を考慮したTwistに変換して保存する。
        """
        self.last_cmd_vel_joy = self.map_joy_twist_to_vehicle_twist(msg)
        self.last_cmd_vel_joy_time = self.get_clock().now()

    def map_joy_twist_to_vehicle_twist(self, msg):
        """
        joy_teleop由来のTwistを、車両的なTwistに変換する。

        前提:
        msg.linear.x  : 前後スティック入力相当
        msg.angular.z : 左右スティック入力相当

        挙動:
        - 正面/背面方向: 直進/後退
        - 斜め方向: 曲がりながら前進/後退
        - joy_max_turn_angle_deg で最大旋回強度に到達
        - joy_max_turn_angle_deg〜joy_side_stop_angle_deg では同じ最大旋回動作を維持
        - joy_side_stop_angle_deg以上、つまりほぼ真横入力では停止
        - 後退時は、スティックを倒した方向へ車体が進むようにyaw符号が自然に反転する
        """
        out = Twist()

        x = float(msg.linear.x)
        y = float(msg.angular.z)

        eps = 1e-6
        r = math.sqrt(x * x + y * y)

        # joy_teleop側でdeadzoneを処理する前提。
        # ここでは数値誤差レベルのみ停止扱いにする。
        if r < eps:
            return out

        # phi: 前後軸から見たスティック角度 [rad]
        phi = math.atan2(abs(y), abs(x))

        max_turn_angle = math.radians(self.joy_max_turn_angle_deg)
        side_stop_angle = math.radians(self.joy_side_stop_angle_deg)

        # ほぼ真横なら停止
        if phi >= side_stop_angle:
            return out

        # 前後方向がほぼゼロの場合も停止
        if abs(x) < eps:
            return out

        # ------------------------
        # 旋回強度
        # ------------------------
        # phi=0deg                  -> 0
        # phi=joy_max_turn_angle_deg -> 1
        # それ以上                 -> 1で飽和
        turn_strength = math.tan(phi) / max(math.tan(max_turn_angle), eps)
        turn_strength = self.clamp(turn_strength, 0.0, 1.0)

        # ------------------------
        # 速度指令
        # ------------------------
        # 重要:
        # joy_max_turn_angle_deg〜joy_side_stop_angle_deg の間で速度を落とさない。
        # スティック倒し量 r のみで速度を決める。
        direction = 1.0 if x > 0.0 else -1.0
        v = direction * r

        # ------------------------
        # 曲率
        # ------------------------
        # y>0: 左旋回, y<0: 右旋回。
        # omega = v * curvature とすることで、
        # 後退時にはyaw rateの符号が自然に反転する。
        turn_sign = 1.0 if y > 0.0 else -1.0
        curvature = turn_sign * turn_strength / max(self.min_turn_radius, eps)

        omega = v * curvature

        out.linear.x = v
        out.angular.z = omega

        return out

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

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

    def apply_motion_limits(self, lin_x, ang_z):
        """
        車両運動制約をTwistに適用する。

        方針:
        1. min_turn_radius は旋回半径制約なので、必要なら ang_z を下げて半径を大きくする
        2. max_yaw_rate はVO保護制約なので、lin_x と ang_z を同率縮小して半径を維持する
        3. max_yaw_accel もVO保護制約なので、lin_x と ang_z を同率縮小して半径を維持する
        """
        eps = 1e-6

        lin_x = float(lin_x)
        ang_z = float(ang_z)

        # ------------------------
        # 0. 停止時はyawもゼロ
        # ------------------------
        if abs(lin_x) < eps:
            ang_z = 0.0
            self.prev_yaw_rate_cmd = 0.0
            self.prev_yaw_rate_time = self.get_clock().now()
            return 0.0, 0.0

        # ------------------------
        # 1. 最小旋回半径制限
        # ------------------------
        # これは速度を落としても解決しないため、
        # ang_zを制限して旋回半径を大きくする。
        max_ang_by_radius = abs(lin_x) / max(self.min_turn_radius, eps)
        ang_z = self.clamp(ang_z, -max_ang_by_radius, max_ang_by_radius)

        # ------------------------
        # 2. 最大yaw rate制限
        # ------------------------
        # ここでは旋回半径を維持するため、
        # lin_x と ang_z を同率縮小する。
        if self.max_yaw_rate > 0.0 and abs(ang_z) > self.max_yaw_rate:
            scale = self.max_yaw_rate / abs(ang_z)
            lin_x *= scale
            ang_z *= scale

        # ------------------------
        # 3. 最大yaw加速度制限
        # ------------------------
        now = self.get_clock().now()

        if self.prev_yaw_rate_time is not None and self.max_yaw_accel > 0.0:
            dt = (now - self.prev_yaw_rate_time).nanoseconds * 1e-9

            if dt > eps:
                max_delta = self.max_yaw_accel * dt
                delta = ang_z - self.prev_yaw_rate_cmd

                if abs(delta) > max_delta:
                    limited_ang_z = self.prev_yaw_rate_cmd + math.copysign(
                        max_delta, delta
                    )

                    # 基本方針:
                    # limited_ang_z が要求yawと同じ符号なら、
                    # lin_x と ang_z を同率縮小して旋回半径を維持する。
                    if abs(ang_z) > eps and (limited_ang_z * ang_z) > 0.0:
                        scale = abs(limited_ang_z) / abs(ang_z)
                        lin_x *= scale
                        ang_z = limited_ang_z

                    else:
                        # 符号反転付近では、半径維持が不安定になりやすい。
                        # 一旦停止扱いにすることで、逆向きの旋回やその場回転を避ける。
                        lin_x = 0.0
                        ang_z = 0.0

        self.prev_yaw_rate_cmd = ang_z
        self.prev_yaw_rate_time = now

        return lin_x, ang_z

    def motor_control(self, cmd):
        """convert command to motor speed and send to ODrive"""

        if cmd is not None:
            lin_x = float(cmd.linear.x)  # velocity (forward/backward)
            ang_z = float(cmd.angular.z)  # yaw rate command

            # 車両運動制約を適用
            lin_x, ang_z = self.apply_motion_limits(lin_x, ang_z)

            wheel_perimeter = self.wheel_radius * 2.0 * math.pi

            # compute rps of L/R wheels
            v_left = (lin_x - ang_z * self.tread_width / 2.0) / wheel_perimeter
            v_right = (lin_x + ang_z * self.tread_width / 2.0) / wheel_perimeter

            # 左右速度比を保ったまま、ホイール最大速度内に収める
            max_abs_whl_rps = max(abs(v_left), abs(v_right))

            if max_abs_whl_rps > self.max_whl_rps:
                scale = self.max_whl_rps / max_abs_whl_rps
                v_left *= scale
                v_right *= scale

            # convert to motor rps
            mtr_left_rps = v_left * self.gear_ratio
            mtr_right_rps = v_right * self.gear_ratio

            # publish the limited equivalent Twist command for simulation
            limited_cmd = Twist()
            limited_cmd.linear.x = lin_x
            limited_cmd.angular.z = ang_z
            self.sim_cmd_vel_pub.publish(limited_cmd)

        else:
            mtr_left_rps = 0.0
            mtr_right_rps = 0.0

            # command timeout時は停止を優先し、yaw ramp状態もリセット
            self.prev_yaw_rate_cmd = 0.0
            self.prev_yaw_rate_time = None

            zero_cmd = Twist()
            self.sim_cmd_vel_pub.publish(zero_cmd)

        if not self.odrive_connected:
            self.left_cmd_rps = 0.0
            self.right_cmd_rps = 0.0
            return

        try:
            # send command to ODrive
            self.left_motor.set_velocity(self.left_motor_sign * mtr_left_rps)
            self.right_motor.set_velocity(self.right_motor_sign * mtr_right_rps)

            # keep command values in vehicle coordinate convention
            self.left_cmd_rps = mtr_left_rps
            self.right_cmd_rps = mtr_right_rps

        except Exception as e:
            self.mark_odrive_disconnected(e)

    def publish_motor_state(self):
        """モータ制御情報を取得しpublishする関数"""
        if not self.odrive_connected:
            return

        try:
            msg = MotorState()

            # オドメトリに使う位置情報を最優先で連続取得する
            position_read_start = self.get_clock().now()

            left_pos_turns = float(
                self.left_motor_sign
                * self.left_motor.get_position()
            )
            right_pos_turns = float(
                self.right_motor_sign
                * self.right_motor.get_position()
            )

            position_read_end = self.get_clock().now()

            # 左右位置取得区間の中点を代表時刻とする
            midpoint_ns = (
                position_read_start.nanoseconds
                + position_read_end.nanoseconds
            ) // 2

            msg.stamp = Time(
                nanoseconds=midpoint_ns,
                clock_type=position_read_start.clock_type,
            ).to_msg()

            msg.left_pos_turns = left_pos_turns
            msg.right_pos_turns = right_pos_turns

            # 速度指令値
            msg.left_cmd_rps = float(self.left_cmd_rps)
            msg.right_cmd_rps = float(self.right_cmd_rps)

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
            msg.right_iq_measured_a = float(
                self.right_motor_sign * self.right_motor.get_iq_measured()
            )
            # q軸電流 [A]　指令値
            msg.left_iq_setpoint_a = float(
                self.left_motor_sign * self.left_motor.get_iq_setpoint()
            )
            msg.right_iq_setpoint_a = float(
                self.right_motor_sign * self.right_motor.get_iq_setpoint()
            )
            '''
            # USBの速度がたりないので一度計測対象外にする
            msg.left_iq_measured_a = 0.0
            msg.right_iq_measured_a = 0.0
            msg.left_iq_setpoint_a = 0.0
            msg.right_iq_setpoint_a = 0.0
            '''

            # 電源電圧
            # ODriveからの実読出しは1 Hzに抑え、それ以外の周期では直近値を再利用する。
            now = self.get_clock().now()
            if (
                self._last_vbus_update_time is None
                or (now - self._last_vbus_update_time).nanoseconds
                >= self._vbus_update_period_sec * 1e9
            ):
                self._vbus_voltage = float(self.left_motor.get_vbus_voltage())
                self._last_vbus_update_time = now

            msg.vbus_voltage = self._vbus_voltage

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
            ("min_turn_radius", self.min_turn_radius),
            ("max_yaw_rate", self.max_yaw_rate),
            ("max_yaw_accel", self.max_yaw_accel),
            ("joy_max_turn_angle_deg", self.joy_max_turn_angle_deg),
            ("joy_side_stop_angle_deg", self.joy_side_stop_angle_deg),
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
        self._last_vbus_update_time = None


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