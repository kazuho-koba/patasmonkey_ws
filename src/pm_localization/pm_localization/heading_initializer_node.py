#!/usr/bin/env python3
"""Calibrate one IMU heading stream and seed local EKF yaw once at startup.

The node deliberately never re-seeds heading while driving.  A runtime heading
reset must reinitialize navsat_transform and global localization together, so
it is an operator workflow rather than an automatic correction.
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_localization.srv import SetDatum, SetPose
from sensor_msgs.msg import Imu, NavSatFix


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def rpy_from_quaternion(q):
    roll = math.atan2(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y),
    )
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return roll, pitch, yaw


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class HeadingInitializerNode(Node):
    """Publish corrected orientation and perform one stationary yaw seed."""

    def __init__(self):
        super().__init__("heading_initializer_node")
        self.declare_parameter("input_imu_topic", "/wit/imu")
        self.declare_parameter("output_imu_topic", "/wit/imu/heading_calibrated")
        self.declare_parameter("wheel_odom_topic", "/wheel/odometry")
        self.declare_parameter("gnss_fix_topic", "/fix")
        self.declare_parameter(
            "local_set_pose_service", "/set_pose"
        )
        self.declare_parameter("yaw_correction_radians", 0.0)
        self.declare_parameter("stationary_speed_threshold_mps", 0.03)
        self.declare_parameter("initial_sample_count", 100)
        self.declare_parameter("max_initial_yaw_stddev_radians", 0.10)
        self.declare_parameter("initialization_timeout_sec", 12.0)
        self.declare_parameter("initial_yaw_variance", 0.05)
        self.declare_parameter(
            "navsat_set_datum_service", "/datum"
        )

        parameter = lambda name: self.get_parameter(name).value
        self.input_imu_topic = str(parameter("input_imu_topic"))
        self.output_imu_topic = str(parameter("output_imu_topic"))
        self.wheel_odom_topic = str(parameter("wheel_odom_topic"))
        self.gnss_fix_topic = str(parameter("gnss_fix_topic"))
        self.local_set_pose_service = str(parameter("local_set_pose_service"))
        self.yaw_correction = float(parameter("yaw_correction_radians"))
        self.stationary_speed_threshold = float(
            parameter("stationary_speed_threshold_mps")
        )
        self.initial_sample_count = int(parameter("initial_sample_count"))
        self.max_initial_yaw_stddev = float(
            parameter("max_initial_yaw_stddev_radians")
        )
        self.initial_yaw_variance = float(parameter("initial_yaw_variance"))
        self.navsat_set_datum_service = str(parameter("navsat_set_datum_service"))

        self.latest_speed = None
        self.sin_sum = 0.0
        self.cos_sum = 0.0
        self.sample_count = 0
        self.initialization_complete = False
        self.initialization_timed_out = False
        self.last_imu_stamp = None
        self.seed_yaw = None
        self.latest_fix = None
        self.datum_sent = False

        self.publisher = self.create_publisher(Imu, self.output_imu_topic, 50)
        self.create_subscription(Imu, self.input_imu_topic, self.imu_callback, 100)
        self.create_subscription(
            Odometry, self.wheel_odom_topic, self.wheel_callback, 20
        )
        self.create_subscription(NavSatFix, self.gnss_fix_topic, self.fix_callback, 10)
        self.set_pose_client = self.create_client(
            SetPose, self.local_set_pose_service
        )
        self.set_datum_client = self.create_client(
            SetDatum, self.navsat_set_datum_service
        )
        timeout = float(parameter("initialization_timeout_sec"))
        self.timeout_timer = self.create_timer(timeout, self.timeout_callback)

        self.get_logger().info(
            "Heading initializer: correction=%+.4f rad; collecting %d "
            "stationary IMU samples before local-EKF yaw seed" % (
                self.yaw_correction, self.initial_sample_count
            )
        )

    def wheel_callback(self, message):
        self.latest_speed = abs(message.twist.twist.linear.x)

    def fix_callback(self, message):
        if (message.status.status < 0 or not all(math.isfinite(value) for value in (
                message.latitude, message.longitude, message.altitude))):
            return
        self.latest_fix = message
        self.try_set_navsat_datum()

    def corrected_message(self, message):
        roll, pitch, raw_yaw = rpy_from_quaternion(message.orientation)
        corrected_yaw = wrap_pi(raw_yaw + self.yaw_correction)
        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, corrected_yaw)
        output = Imu()
        output.header = message.header
        output.orientation.x = qx
        output.orientation.y = qy
        output.orientation.z = qz
        output.orientation.w = qw
        output.orientation_covariance = message.orientation_covariance
        output.angular_velocity = message.angular_velocity
        output.angular_velocity_covariance = message.angular_velocity_covariance
        output.linear_acceleration = message.linear_acceleration
        output.linear_acceleration_covariance = message.linear_acceleration_covariance
        return output, corrected_yaw

    def imu_callback(self, message):
        output, yaw = self.corrected_message(message)
        self.publisher.publish(output)
        self.last_imu_stamp = output.header.stamp
        if self.initialization_complete or self.initialization_timed_out:
            return
        if self.latest_speed is None:
            return
        if self.latest_speed > self.stationary_speed_threshold:
            self.reset_samples("wheel speed %.3f m/s is not stationary" % self.latest_speed)
            return
        self.sin_sum += math.sin(yaw)
        self.cos_sum += math.cos(yaw)
        self.sample_count += 1
        if self.sample_count >= self.initial_sample_count:
            self.seed_local_ekf()

    def reset_samples(self, reason):
        if self.sample_count:
            self.get_logger().warn("Discarding heading samples: %s" % reason)
        self.sin_sum = 0.0
        self.cos_sum = 0.0
        self.sample_count = 0

    def timeout_callback(self):
        if self.initialization_complete:
            return
        self.initialization_timed_out = True
        self.timeout_timer.cancel()
        self.get_logger().error(
            "Heading seed was not applied within the startup window "
            "(samples=%d, latest_speed=%s, set_pose_ready=%s). Keep the "
            "vehicle stationary during initialization, then restart the "
            "localization launch before driving." % (
                self.sample_count,
                "none" if self.latest_speed is None else "%.3f" % self.latest_speed,
                self.set_pose_client.service_is_ready(),
            )
        )

    def seed_local_ekf(self):
        mean_yaw = math.atan2(self.sin_sum, self.cos_sum)
        concentration = math.hypot(self.sin_sum, self.cos_sum) / self.sample_count
        # Circular standard deviation, robust for an angle near +/- pi.
        stddev = math.sqrt(max(0.0, -2.0 * math.log(max(concentration, 1e-12))))
        if stddev > self.max_initial_yaw_stddev:
            self.reset_samples(
                "heading standard deviation %.3f rad exceeds %.3f rad" % (
                    stddev, self.max_initial_yaw_stddev
                )
            )
            return
        if not self.set_pose_client.service_is_ready():
            self.get_logger().info("Waiting for local EKF set_pose service")
            return
        request = SetPose.Request()
        pose = PoseWithCovarianceStamped()
        # Use the sensor stamp, which keeps replayed set_pose requests in the
        # EKF's simulated time domain without requiring this node to process a
        # high-rate /clock timer for every recorded sensor message.
        pose.header.stamp = self.last_imu_stamp or self.get_clock().now().to_msg()
        pose.header.frame_id = "odom"
        qx, qy, qz, qw = quaternion_from_rpy(0.0, 0.0, mean_yaw)
        pose.pose.pose.orientation.x = qx
        pose.pose.pose.orientation.y = qy
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw
        pose.pose.covariance[0] = 1e-6
        pose.pose.covariance[7] = 1e-6
        pose.pose.covariance[14] = 1e6
        pose.pose.covariance[21] = 1e6
        pose.pose.covariance[28] = 1e6
        pose.pose.covariance[35] = self.initial_yaw_variance
        request.pose = pose
        self.set_pose_client.call_async(request)
        self.initialization_complete = True
        self.seed_yaw = mean_yaw
        self.timeout_timer.cancel()
        self.get_logger().info(
            "Seeded local EKF yaw once: %.3f rad (circular stddev %.3f rad)" % (
                mean_yaw, stddev
            )
        )
        self.try_set_navsat_datum()

    def try_set_navsat_datum(self):
        """Release navsat_transform only after a valid stationary heading seed."""
        if (not self.initialization_complete or self.latest_fix is None
                or self.datum_sent or not self.set_datum_client.service_is_ready()):
            return
        request = SetDatum.Request()
        request.geo_pose.position.latitude = self.latest_fix.latitude
        request.geo_pose.position.longitude = self.latest_fix.longitude
        request.geo_pose.position.altitude = self.latest_fix.altitude
        qx, qy, qz, qw = quaternion_from_rpy(0.0, 0.0, self.seed_yaw)
        request.geo_pose.orientation.x = qx
        request.geo_pose.orientation.y = qy
        request.geo_pose.orientation.z = qz
        request.geo_pose.orientation.w = qw
        self.set_datum_client.call_async(request)
        self.datum_sent = True
        self.get_logger().info(
            "Sent navsat datum after stationary heading initialization"
        )


def main(args=None):
    rclpy.init(args=args)
    node = HeadingInitializerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
