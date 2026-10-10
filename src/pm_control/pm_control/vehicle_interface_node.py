import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from pm_msgs.msg import MotorState
from rclpy.time import Time
from .odrive_controller import MotorController
import math
import sys
import json
import time


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
        self.declare_parameter("max_linear_speed", 1.0)  # 全モードの絶対速度上限 [m/s]
        self.declare_parameter("joy_normal_max_linear_speed", 1.0)
        self.declare_parameter("joy_turbo_max_linear_speed", 2.0)
        self.declare_parameter("autonomous_max_linear_speed", 1.0)
        self.declare_parameter("joy_state_topic", "/pm/joy")
        self.declare_parameter("joy_turbo_button", 1)

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

        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.joy_normal_max_linear_speed = float(
            self.get_parameter("joy_normal_max_linear_speed").value)
        self.joy_turbo_max_linear_speed = float(
            self.get_parameter("joy_turbo_max_linear_speed").value)
        self.autonomous_max_linear_speed = float(
            self.get_parameter("autonomous_max_linear_speed").value)
        self.joy_state_topic = str(self.get_parameter("joy_state_topic").value)
        self.joy_turbo_button = int(self.get_parameter("joy_turbo_button").value)
        self._joy_turbo_active = False
        self._joy_state_received_at = None
        # 不正な寸法・閾値を接続前に拒否し、ゼロ除算や内側車輪の逆転を避ける。
        positive = (self.wheel_radius, self.tread_width, self.gear_ratio,
                    self.max_whl_rps, self.max_linear_speed, self.min_turn_radius,
                    self.joy_normal_max_linear_speed, self.joy_turbo_max_linear_speed,
                    self.autonomous_max_linear_speed)
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("Vehicle geometry and speed limits must be finite and positive")
        if self.min_turn_radius <= self.tread_width / 2.0:
            raise ValueError("min_turn_radius must exceed tread_width / 2")
        if any(not math.isfinite(value) or value < 0.0
               for value in (self.max_yaw_rate, self.max_yaw_accel)):
            raise ValueError("Yaw limits must be finite and nonnegative")
        if not (0.0 < self.joy_max_turn_angle_deg
                < self.joy_side_stop_angle_deg <= 90.0):
            raise ValueError("Joystick angles must satisfy 0 < turn < stop <= 90")

        # display the set parameters
        self.print_parameters()

        # 走行用launchだけが中立インターロックを使用する。Core起動では本nodeを起動しない。
        self.declare_parameter('require_neutral_on_start', False)
        self.declare_parameter('vehicle_status_topic', '/pm/vehicle/status')
        self.require_neutral = bool(self.get_parameter('require_neutral_on_start').value)
        self._drive_armed = not self.require_neutral
        self._emergency_stop_active = False
        self._neutral_received_at = None
        self._connected_at = None
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('vehicle_status_topic').value), 10)
        self.create_timer(0.5, self.publish_vehicle_status)

        # connect to odrive
        self.left_motor = None
        self.right_motor = None
        self.odrive_connected = False
        self.reconnect_in_progress = False

        self.reset_motion_limits()
        self.connect_odrive()
        self._reconnect_timer = self.create_timer(1.0, self.try_reconnect_odrive)

        # self.left_motor.get_velocity()

        # param definition related to subscriptions
        self.last_cmd_vel = None
        self.last_cmd_vel_time = None
        self.last_cmd_vel_joy = None
        self.last_cmd_vel_joy_time = None

        # Twistだけでは通常の斜め入力とTurboを区別できないため、既存Joyのボタンを監視する。
        # sensor QoSはbest_effort/reliable双方のJoy publisherから受信できる。
        self.create_subscription(Joy, self.joy_state_topic,
                                 self.joy_state_callback, qos_profile_sensor_data)
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
            self._drive_armed = not self.require_neutral and not self._emergency_stop_active
            self._neutral_received_at = None
            self.get_logger().info("connecting to ODrive...")
            self.left_motor = MotorController(
                self.mtr_axis_l,
                vel_ramp_rate=self.vel_ramp_rate,
                pos_gain=self.pos_gain,
                vel_gain=self.vel_gain,
                vel_integrator_gain=self.vel_integrator_gain,
                vel_integrator_limit=self.vel_integrator_limit,
                activate=not self.require_neutral and not self._emergency_stop_active,
            )

            self.right_motor = MotorController(
                self.mtr_axis_r,
                vel_ramp_rate=self.vel_ramp_rate,
                pos_gain=self.pos_gain,
                vel_gain=self.vel_gain,
                vel_integrator_gain=self.vel_integrator_gain,
                vel_integrator_limit=self.vel_integrator_limit,
                activate=not self.require_neutral and not self._emergency_stop_active,
            )

            self.left_cmd_rps = 0.0  # モータ指令値をpublishするために値を保存しておく変数（左）
            self.right_cmd_rps = 0.0  # モータ指令値をpublishするために値を保存しておく変数（右）
            self.reset_motion_limits()
            self.odrive_connected = True
            self._connected_at = time.monotonic()
            self.reconnect_in_progress = False

            # 再接続後の最初のMotorState publishでvbusを即時再取得する
            self._last_vbus_update_time = None

            self.get_logger().info("ODrive connected and initialized!")

        except Exception as e:
            # 右軸の接続失敗時にも先に接続した左軸をIDLEへ戻す。
            self.stop_motors()
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
        joy_teleop由来の生Twistと鮮度を保存し、中立インターロックを更新する。
        """
        # 車両変換後のゼロは真横入力でも発生するため、生Twistの中立だけを使う。
        values = (float(msg.linear.x), float(msg.angular.z))
        if all(math.isfinite(v) and abs(v) < 1e-6 for v in values):
            self._neutral_received_at = time.monotonic()
        else:
            self._neutral_received_at = None
        # 元のTwistを保持し、制御周期ごとに最新のボタン状態で変換する。
        # ボタンを離した後も古いTurbo変換済み指令が残ることを避ける。
        self.last_cmd_vel_joy = msg
        self.last_cmd_vel_joy_time = self.get_clock().now()

    def joy_state_callback(self, msg):
        """既存JoyのTurboボタンと受信時刻を保持する。欠損・無効番号は通常扱い。"""
        index = self.joy_turbo_button
        self._joy_turbo_active = (
            0 <= index < len(msg.buttons) and bool(msg.buttons[index]))
        self._joy_state_received_at = time.monotonic()

    def joy_speed_limit(self):
        """直近0.3秒の明示的なTurbo入力だけで高速側を許可する。"""
        received = self._joy_state_received_at
        if (self._joy_turbo_active and received is not None
                and 0.0 <= time.monotonic() - received < 0.3):
            return min(self.max_linear_speed, self.joy_turbo_max_linear_speed)
        return min(self.max_linear_speed, self.joy_normal_max_linear_speed)

    def map_joy_twist_to_vehicle_twist(self, msg, speed_limit=None):
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
        # 入力の異常値は停止扱いとし、通常・turbo・斜め入力を共通速度上限内に収める。
        if not (math.isfinite(x) and math.isfinite(y)):
            return out
        # modeごとの上限でノルムを丸め、通常の斜め入力をTurboと誤判定しない。
        if speed_limit is None:
            speed_limit = self.joy_normal_max_linear_speed
        r = min(math.hypot(x, y), speed_limit, self.max_linear_speed)

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
        # この段階では倒し量から希望速度を作る。旋回時の減速と時間方向の平滑化は
        # apply_motion_limitsでyaw制約と曲率から計算し、手動・自律指令へ共通適用する。
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
            if self._emergency_stop_active:
                return
            if not self._drive_armed:
                # 再接続後にも新しい中立を要求する。ROS時刻停止中も鮮度を延長しない。
                neutral = self._neutral_received_at
                if (self.odrive_connected and neutral is not None
                        and self._connected_at is not None and neutral >= self._connected_at
                        and time.monotonic() - neutral < 0.3):
                    try:
                        self.left_motor.activate()
                        self.right_motor.activate()
                    except Exception:
                        self.stop_motors()
                        raise
                    self._drive_armed = True
                    self.last_cmd_vel = None
                    self.last_cmd_vel_time = None
                    self.reset_motion_limits()
                    self.get_logger().info('中立指令確認: 走行出力を許可しました')
                return
            now = self.get_clock().now()
            cmd = None
            speed_limit = min(self.max_linear_speed, self.autonomous_max_linear_speed)

            # prioritize /cmd_vel_joy from gamepad
            if self.last_cmd_vel_joy is not None:
                # check the command's newness
                if (now - self.last_cmd_vel_joy_time).nanoseconds < 0.3 * 1e9:
                    speed_limit = self.joy_speed_limit()
                    cmd = self.map_joy_twist_to_vehicle_twist(
                        self.last_cmd_vel_joy, speed_limit)

            # use /cmd_vel when no joy cmd received
            if cmd is None and self.last_cmd_vel is not None:
                # check the command's newness
                if (now - self.last_cmd_vel_time).nanoseconds < 0.3 * 1e9:
                    cmd = self.last_cmd_vel

            # control motor:
            self.motor_control(cmd, speed_limit)

        except Exception as e:
            self.get_logger().error(f"Exception in command_selector: {e}")

    def reset_motion_limits(self):
        """停止・切断・再接続時に最終指令の履歴をゼロへ戻す。"""
        self.prev_linear_speed_cmd = 0.0
        self.prev_yaw_rate_cmd = 0.0
        self.prev_yaw_rate_time = self.get_clock().now()

    def apply_motion_limits(self, lin_x, ang_z, speed_limit=None):
        """曲率とyaw制約から速度を適応的に落とし、速度とyawを独立に平滑化する。

        入力は車体中心速度[m/s]とyawレート[rad/s]。目標曲率を保持した定常速度を
        yaw上限との滑らかな最小値で求め、速度の変化率はmax_yaw_accel×Rmin[m/s²]
        から導く。過渡では曲率を厳密維持せず、yaw制約のために速度を増幅しない。
        停止・前後反転・異常入力、および速度/yaw/半径の絶対制限は変化率より優先する。
        """
        eps = 1e-6
        lin_x, ang_z = float(lin_x), float(ang_z)
        # mode上限は絶対上限以下。Turbo解除時は速度ランプより通常上限を優先する。
        speed_limit = (self.max_linear_speed if speed_limit is None
                       else min(self.max_linear_speed, speed_limit))
        now = self.get_clock().now()
        if not (math.isfinite(lin_x) and math.isfinite(ang_z)) or abs(lin_x) < eps:
            self.reset_motion_limits()
            return 0.0, 0.0

        # 正転中の後退要求などは、一度ゼロ指令を挟んで履歴をリセットする。
        if lin_x * self.prev_linear_speed_cmd < 0.0:
            self.reset_motion_limits()
            return 0.0, 0.0

        curvature = self.clamp(ang_z / lin_x,
                               -1.0 / self.min_turn_radius,
                               1.0 / self.min_turn_radius)
        target_v = self.clamp(lin_x, -speed_limit, speed_limit)
        if self.max_yaw_rate > 0.0:
            # p=4の滑らかな最小値。直進では減速せず、曲率が強いほど
            # |v|をmax_yaw_rate/|curvature|以下へ滑らかに近づける。
            ratio = abs(target_v * curvature) / self.max_yaw_rate
            target_v /= math.hypot(1.0, ratio * ratio) ** 0.5

        # 遅延や時計巻き戻りで許容量を拡大しない。初回もゼロ履歴から立ち上げる。
        dt = self.clamp((now - self.prev_yaw_rate_time).nanoseconds * 1e-9, 0.0, 0.1)
        if self.max_yaw_accel > 0.0:
            delta_v = self.max_yaw_accel * self.min_turn_radius * dt
            lin_x = self.clamp(target_v, self.prev_linear_speed_cmd - delta_v,
                               self.prev_linear_speed_cmd + delta_v)
        else:
            lin_x = target_v
        lin_x = self.clamp(lin_x, -speed_limit, speed_limit)

        target_w = lin_x * curvature
        if self.max_yaw_rate > 0.0:
            target_w = self.clamp(target_w, -self.max_yaw_rate, self.max_yaw_rate)
        if self.max_yaw_accel > 0.0:
            delta_w = self.max_yaw_accel * dt
            ang_z = self.clamp(target_w, self.prev_yaw_rate_cmd - delta_w,
                               self.prev_yaw_rate_cmd + delta_w)
        else:
            ang_z = target_w

        # 減速中に残るyawが小さな半径やその場旋回を作らないよう、最後に絶対制限する。
        # 実現不可能な組み合わせではyaw変化率より半径・yaw上限・停止を優先する。
        yaw_cap = abs(lin_x) / self.min_turn_radius
        if self.max_yaw_rate > 0.0:
            yaw_cap = min(yaw_cap, self.max_yaw_rate)
        ang_z = self.clamp(ang_z, -yaw_cap, yaw_cap)
        self.prev_linear_speed_cmd = lin_x
        self.prev_yaw_rate_cmd = ang_z
        self.prev_yaw_rate_time = now
        return lin_x, ang_z

    def motor_control(self, cmd, speed_limit=None):
        """Twist[m/s, rad/s]を左右モータ回転数[rps]へ変換して送る。

        車体制約、差動駆動変換、車輪回転数上限の順で処理する。最終車輪指令から
        逆算したTwistを履歴とsim出力へ反映し、タイムアウト時はゼロ指令を送る。
        """

        if cmd is not None:
            lin_x = float(cmd.linear.x)  # velocity (forward/backward)
            ang_z = float(cmd.angular.z)  # yaw rate command

            # 車両運動制約を適用
            lin_x, ang_z = self.apply_motion_limits(lin_x, ang_z, speed_limit)

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

            # 車輪の絶対制限後に車体Twistを逆算し、次周期の履歴とsim出力を一致させる。
            lin_x = (v_left + v_right) * wheel_perimeter / 2.0
            ang_z = (v_right - v_left) * wheel_perimeter / self.tread_width
            self.prev_linear_speed_cmd = lin_x
            self.prev_yaw_rate_cmd = ang_z
            limited_cmd = Twist()
            limited_cmd.linear.x = lin_x
            limited_cmd.angular.z = ang_z
            self.sim_cmd_vel_pub.publish(limited_cmd)

        else:
            mtr_left_rps = 0.0
            mtr_right_rps = 0.0

            # 指令タイムアウトは停止を優先し、再開時もゼロ履歴から立ち上げる。
            self.reset_motion_limits()

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

            # 左右軸が共有するODrive全体のDC入力電流を、30 Hzのpublishごとに1回読む。
            # 消費/回生の符号をそのまま保持し、motor_signを掛けたり左右加算したりしない。
            # stampは上記の位置取得代表時刻であり、電流の厳密な個別取得時刻ではない。
            msg.ibus_a = float(self.left_motor.get_bus_current())

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
        """緊急停止は再起動までラッチする。GUIの走行許可とは別の安全経路。"""
        if msg.data:
            self._emergency_stop_active = True
            self.reset_motion_limits()
            self.left_cmd_rps = self.right_cmd_rps = 0.0
            self.stop_motors()
            self.get_logger().warn("Emergency STOP activated!")

    def stop_motors(self):
        """正常終了でも各軸へゼロとIDLEを送る。片軸の失敗で他軸を飛ばさない。"""
        self._drive_armed = False
        succeeded = True
        for motor in (self.left_motor, self.right_motor):
            if motor is None:
                continue
            try:
                motor.set_velocity(0.0)
            except Exception as exc:
                succeeded = False
                self.get_logger().error('停止ゼロ指令に失敗: {}'.format(exc))
            try:
                # set_idleの旧実装は例外を握りつぶすため、ここでは結果を隠さない。
                from odrive.enums import AXIS_STATE_IDLE
                motor.axis.requested_state = AXIS_STATE_IDLE
            except Exception as exc:
                succeeded = False
                self.get_logger().error('ODrive IDLE指令に失敗: {}'.format(exc))

        return succeeded

    def publish_vehicle_status(self):
        """Managerへ接続・中立待ち・緊急停止を通知。実際の回転有無とは区別する。"""
        message = String()
        message.data = json.dumps({
            'connected': self.odrive_connected,
            'armed': self._drive_armed and self.odrive_connected,
            'emergency_stop': self._emergency_stop_active,
        })
        self._status_pub.publish(message)

    def print_parameters(self):
        header = f"{'Parameter':<20} {'Value':<20}"
        separator = "-" * 42
        lines = [header, separator]
        for key, value in [
            ("wheel_radius", self.wheel_radius),
            ("tread_width", self.tread_width),
            ("gear_ratio", self.gear_ratio),
            ("max_whl_rps", self.max_whl_rps),
            ("max_linear_speed", self.max_linear_speed),
            ("joy_normal_max_linear_speed", self.joy_normal_max_linear_speed),
            ("joy_turbo_max_linear_speed", self.joy_turbo_max_linear_speed),
            ("autonomous_max_linear_speed", self.autonomous_max_linear_speed),
            ("joy_state_topic", self.joy_state_topic),
            ("joy_turbo_button", self.joy_turbo_button),
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

        self.stop_motors()
        self.reset_motion_limits()
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
        # SIGINT後にROS contextが無効でもUSB経由の停止処理を必ず実行する。
        stopped = node.stop_motors()
        node.destroy_node()
        if rclpy.ok():  # Prevent multiple shutdown calls
            rclpy.shutdown()  # Shutdown ROS2 cleanly
        # USB停止指令の失敗はnodeの終了コードにも反映する。詳細はjournalへ残す。
        sys.exit(0 if stopped else 1)


if __name__ == "__main__":
    main()
