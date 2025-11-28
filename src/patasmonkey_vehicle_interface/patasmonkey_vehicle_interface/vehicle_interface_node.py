import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray
from .odrive_controller import MotorController
import math
import sys


class VehicleInterfaceNode(Node):
    def __init__(self):
        super().__init__("vehicle_interface")  # register the node

        # load parameters from yaml (passed via launch file)
        self.whl_rad = self.get_parameter_or("whl_rad", 0.1)  # wheel radius [m]
        self.whl_sep = self.get_parameter_or(
            "whl_sep", 0.4
        )  # wheel separation betwee L\R [m]
        self.gear_ratio = self.get_parameter_or("gear_ratio", 10.0)  # gear ratio
        self.max_whl_rps = self.get_parameter_or(
            "max_whl_rps", 4.0
        )  # max wheel velocity [rps]

        # get odrive config
        self.odrv_usb_port = self.get_parameter_or("odrv_usb_port", "/dev/ttyACM0")
        self.odrv_baud_rate = self.get_parameter_or("odrv_baud_rate", 115200)

        # odrive axis
        self.mtr_axis_l = self.get_parameter_or("mtr_axis_l", 0)
        self.mtr_axis_r = self.get_parameter_or("mtr_axis_r", 1)

        # get topic name that will be used
        self.cmd_vel_topic = self.get_parameter_or("cmd_vel_topic", "/cmd_vel")
        self.cmd_vel_joy_topic = self.get_parameter_or(
            "cmd_vel_joy_topic", "/cmd_vel_joy"
        )
        self.mtr_output_topic = self.get_parameter_or("mtr_output_topic", "/motor_cmd")
        self.emergency_stop_topic = self.get_parameter_or(
            "emergency_stop_topic", "/emergency_stop"
        )

        # display the set parameters
        self.print_parameters()

        # connect to odrive
        self.get_logger().info("connecting to ODrive...")
        self.left_motor = MotorController(self.mtr_axis_l)
        self.right_motor = MotorController(self.mtr_axis_r)
        self.get_logger().info("ODrive connected!")
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

        # other params
        self.current_vel_left = 0.0
        self.last_vel_left = 0.0
        self.current_vel_right = 0.0
        self.last_vel_right = 0.0
        self.accumerated_ver_err_left = 0.0
        self.accumerated_ver_err_right = 0.0

        # timer for control loop
        self._timer = self.create_timer(0.02, self.command_selector)

        # publihser config
        self.motor_cmd_pub = self.create_publisher(
            Float32MultiArray, self.mtr_output_topic, 10
        )
        self.sim_cmd_vel_pub = self.create_publisher(Twist, "/sim_cmd_vel", 10)

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
            whl_diam = self.whl_rad * 2 * math.pi  # wheel diameter

            # compute rps of L/R wheels
            v_left = (lin_x - ang_z * self.whl_sep / 2) / whl_diam
            v_right = (lin_x + ang_z * self.whl_sep / 2) / whl_diam

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

        # send command to ODrive
        self.left_motor.set_velocity(mtr_left_rps)
        self.right_motor.set_velocity(mtr_right_rps)

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

        # publish /motor_cmd
        msg_out = Float32MultiArray()
        msg_out.data = [mtr_left_rps, mtr_right_rps]
        self.motor_cmd_pub.publish(msg_out)

    def emergency_stop_callback(self, msg):
        """emefgency stop: stop motors immediately"""
        if msg.data:
            self.left_motor.set_idle()
            self.right_motor.set_idle()
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
            ("whl_rad", self.whl_rad),
            ("whl_sep", self.whl_sep),
            ("gear_ratio", self.gear_ratio),
            ("max_whl_rps", self.max_whl_rps),
            ("odrv_usb_port", self.odrv_usb_port),
            ("odrv_baud_rate", self.odrv_baud_rate),
            ("mtr_axis_l", self.mtr_axis_l),
            ("mtr_axis_r", self.mtr_axis_r),
            ("cmd_vel_topic", self.cmd_vel_topic),
            ("cmd_vel_joy_topic", self.cmd_vel_joy_topic),
            ("mtr_output_topic", self.mtr_output_topic),
            ("emergency_stop_topic", self.emergency_stop_topic),
        ]:
            lines.append(f"{key:<20} {str(value):<20}")
        self.get_logger().info("\n".join(lines))


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
