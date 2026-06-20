#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)


def quat_to_rot(q):
    """
    geometry_msgs Quaternion -> 3x3 rotation matrix.
    q order in ROS msg: x, y, z, w
    """
    x, y, z, w = q.x, q.y, q.z, q.w

    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)

    s = 2.0 / n

    xx = x * x * s
    yy = y * y * s
    zz = z * z * s
    xy = x * y * s
    xz = x * z * s
    yz = y * z * s
    wx = w * x * s
    wy = w * y * s
    wz = w * z * s

    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=float,
    )


def rot_to_quat(R):
    """
    3x3 rotation matrix -> quaternion tuple (x, y, z, w)
    """
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]

    tr = m00 + m11 + m22

    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif (m00 > m11) and (m00 > m22):
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return 0.0, 0.0, 0.0, 1.0

    return x / norm, y / norm, z / norm, w / norm


def rpy_from_rot(R):
    """
    Rotation matrix -> roll, pitch, yaw.
    ROS convention approximation:
      roll  around x
      pitch around y
      yaw   around z
    """
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    if sy > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0

    return roll, pitch, yaw


def rad2deg(x):
    return x * 180.0 / math.pi


def transform_to_matrix(t: TransformStamped):
    """
    geometry_msgs TransformStamped -> 4x4 matrix.
    Returned matrix represents T_target_source.
    """
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(t.transform.rotation)
    T[:3, 3] = np.array(
        [
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ],
        dtype=float,
    )
    return T


def odom_pose_to_matrix(msg: Odometry):
    """
    nav_msgs/Odometry pose -> 4x4 matrix.
    Interpreted as T_header_child.
    """
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(msg.pose.pose.orientation)
    T[:3, 3] = np.array(
        [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
        dtype=float,
    )
    return T


def openvins_odom_pose_to_matrix(msg: Odometry, invert_orientation: bool):
    """
    Convert OpenVINS Odometry pose to 4x4 matrix.

    Position is assumed to be p_imu_in_global.

    If invert_orientation is True:
      msg.orientation is interpreted as R_imu_global, i.e. q_GtoI-like,
      and converted to R_global_imu by transpose.

    If invert_orientation is False:
      msg.orientation is interpreted directly as R_global_imu.
    """
    T = np.eye(4)

    R_msg = quat_to_rot(msg.pose.pose.orientation)

    if invert_orientation:
        R_global_imu = R_msg.T
    else:
        R_global_imu = R_msg

    T[:3, :3] = R_global_imu
    T[:3, 3] = np.array(
        [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ],
        dtype=float,
    )

    return T


def skew(v):
    """
    Return skew-symmetric matrix [v]x such that [v]x @ w = v x w.
    """
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=float,
    )


def cov_list_to_mat6(cov_list):
    """
    ROS covariance list, row-major length 36 -> 6x6 numpy matrix.
    Ordering is assumed to be:
      [x, y, z, rot_x, rot_y, rot_z]
    """
    return np.array(cov_list, dtype=float).reshape((6, 6))


def cov_mat6_to_list(P):
    """
    6x6 numpy covariance matrix -> ROS covariance list.
    Symmetrize to avoid small numerical asymmetry.
    """
    P = 0.5 * (P + P.T)
    return P.reshape(-1).astype(float).tolist()


def transform_pose_covariance_ros_approx(
    pose_cov,
    R_odom_global,
    R_global_imu,
    p_base_in_imu,
):
    """
    Approximate transform of ROS Odometry pose covariance.

    Input covariance:
      Pose covariance of imu pose in global frame.
      Ordering: [x, y, z, rot_x, rot_y, rot_z]

    Pose transform:
      p_global_base = p_global_imu + R_global_imu * p_imu_base
      p_odom_base   = R_odom_global * p_global_base + t

    Small-angle approximation:
      δp_odom_base =
          R_odom_global * δp_global_imu
        - R_odom_global * skew(R_global_imu * p_base_in_imu) * δtheta_global_imu

      δtheta_odom_base =
          R_odom_global * δtheta_global_imu

    Therefore:
      P_out = J * P_in * J.T
    """
    P_in = cov_list_to_mat6(pose_cov)

    lever_global = R_global_imu @ p_base_in_imu

    J = np.zeros((6, 6), dtype=float)
    J[0:3, 0:3] = R_odom_global
    J[0:3, 3:6] = -R_odom_global @ skew(lever_global)
    J[3:6, 3:6] = R_odom_global

    P_out = J @ P_in @ J.T
    return cov_mat6_to_list(P_out)


def transform_twist_and_covariance_ros_approx(
    twist,
    twist_cov,
    R_base_imu,
    p_imu_in_base,
):
    """
    Transform twist from imu frame/origin to base_link frame/origin.

    Assumption:
      input twist is expressed in imu frame at imu origin:
        v_imu_origin_in_imu
        w_imu_in_imu

      output twist is expressed in base_link frame at base_link origin:
        v_base_origin_in_base
        w_base_in_base

    Kinematics:
      w_base = R_base_imu * w_imu

      Let p_imu_in_base be vector from base origin to imu origin, expressed in base.
      v_imu_origin = v_base_origin + w_base x p_imu_in_base
      therefore:
      v_base_origin = R_base_imu * v_imu - w_base x p_imu_in_base
                    = R_base_imu * v_imu + skew(p_imu_in_base) * R_base_imu * w_imu

    Jacobian:
      [v_base]   [ R_base_imu   skew(p_imu_in_base) R_base_imu ] [v_imu]
      [w_base] = [     0                 R_base_imu             ] [w_imu]
    """
    v_imu = np.array(
        [
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
        ],
        dtype=float,
    )

    w_imu = np.array(
        [
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        ],
        dtype=float,
    )

    w_base = R_base_imu @ w_imu
    v_base = R_base_imu @ v_imu + skew(p_imu_in_base) @ w_base

    P_in = cov_list_to_mat6(twist_cov)

    J = np.zeros((6, 6), dtype=float)
    J[0:3, 0:3] = R_base_imu
    J[0:3, 3:6] = skew(p_imu_in_base) @ R_base_imu
    J[3:6, 3:6] = R_base_imu

    P_out = J @ P_in @ J.T

    return v_base, w_base, cov_mat6_to_list(P_out)


class VioOdomAdapterNode(Node):
    """
    Convert OpenVINS odometry:
      /ov_msckf/odomimu, frame_id=global, child_frame_id=imu

    into robot odometry:
      /vio/odometry, frame_id=odom, child_frame_id=base_link

    Mathematical model:
      T_global_imu  : from OpenVINS
      T_base_imu    : from TF lookup base_link <- oakd_imu_link
      T_imu_base    : inverse(T_base_imu)

      T_global_base = T_global_imu * T_imu_base

    If zero_initial_pose is true:
      T_odom_global = inverse(T_global_base at first message)
      T_odom_base   = T_odom_global * T_global_base
    """

    def __init__(self):
        super().__init__("vio_odom_adapter_node")

        self.declare_parameter("input_topic", "/ov_msckf/odomimu")
        self.declare_parameter("output_topic", "/vio/odometry")

        self.declare_parameter("output_frame_id", "odom")
        self.declare_parameter("output_child_frame_id", "base_link")

        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("oak_imu_frame_id", "oakd_imu_link")

        self.declare_parameter("zero_initial_pose", True)
        self.declare_parameter("publish_tf", False)

        self.declare_parameter("invert_openvins_orientation", True)
        self.declare_parameter("align_initial_to_tf", True)

        self.declare_parameter("enable_diagnostics", False)
        self.declare_parameter("diagnostics_interval_sec", 1.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        self.output_frame_id = self.get_parameter("output_frame_id").value
        self.output_child_frame_id = self.get_parameter("output_child_frame_id").value

        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.oak_imu_frame_id = self.get_parameter("oak_imu_frame_id").value

        self.zero_initial_pose = bool(self.get_parameter("zero_initial_pose").value)
        self.invert_openvins_orientation = bool(
            self.get_parameter("invert_openvins_orientation").value
        )
        self.align_initial_to_tf = bool(self.get_parameter("align_initial_to_tf").value)

        self.enable_diagnostics = bool(self.get_parameter("enable_diagnostics").value)
        self.diagnostics_interval_sec = float(
            self.get_parameter("diagnostics_interval_sec").value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.T_base_imu = None
        self.T_imu_base = None
        self.T_odom_global = None

        self.T_global_imu_first = None
        self.T_global_base_first = None
        self.T_odom_base_first = None
        self.last_diag_time = self.get_clock().now()

        self.pub = self.create_publisher(Odometry, self.output_topic, 10)
        self.sub = self.create_subscription(
            Odometry, self.input_topic, self.odom_callback, 50
        )

        self.get_logger().info(
            f"VIO odom adapter started: {self.input_topic} -> {self.output_topic}"
        )
        self.get_logger().info(
            f"Using static transform: {self.base_frame_id} -> {self.oak_imu_frame_id}"
        )

    def try_update_static_transform(self):
        if self.T_base_imu is not None:
            return True

        try:
            # lookup_transform(target, source, time)
            # This returns T_target_source.
            tf_msg = self.tf_buffer.lookup_transform(
                self.base_frame_id, self.oak_imu_frame_id, rclpy.time.Time()
            )
            self.T_base_imu = transform_to_matrix(tf_msg)
            self.T_imu_base = np.linalg.inv(self.T_base_imu)

            self.get_logger().info(
                f"Got TF: {self.base_frame_id} -> {self.oak_imu_frame_id}"
            )
            return True

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"Waiting for TF {self.base_frame_id} -> {self.oak_imu_frame_id}: {str(e)}",
                throttle_duration_sec=2.0,
            )
            return False

    def try_initialize_odom_global(self, T_global_base):
        """
        Initialize T_odom_global.

        Preferred:
          align first VIO pose to current EKF TF: odom -> base_link

        Fallback:
          if zero_initial_pose is true, make first VIO pose identity.
        """
        if self.T_odom_global is not None:
            return True

        if self.align_initial_to_tf:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.output_frame_id, self.output_child_frame_id, rclpy.time.Time()
                )
                T_odom_base_ref = transform_to_matrix(tf_msg)

                self.T_odom_global = T_odom_base_ref @ np.linalg.inv(T_global_base)

                self.get_logger().info(
                    f"Initialized T_odom_global from TF "
                    f"{self.output_frame_id} -> {self.output_child_frame_id}"
                )
                return True

            except (
                LookupException,
                ConnectivityException,
                ExtrapolationException,
            ) as e:
                self.get_logger().warn(
                    f"Waiting for initial TF "
                    f"{self.output_frame_id} -> {self.output_child_frame_id}: {str(e)}",
                    throttle_duration_sec=2.0,
                )
                return False

        if self.zero_initial_pose:
            self.T_odom_global = np.eye(4)
            self.T_odom_global[:3, 3] = -T_global_base[:3, 3]
            self.get_logger().info(
                "Initialized T_odom_global with translation-only zeroing"
            )   
        else:
            self.T_odom_global = np.eye(4)
            self.get_logger().info("Initialized T_odom_global as identity")

        return True


    def maybe_print_diagnostics(self, msg, T_global_imu, T_global_base, T_odom_base):
        if not self.enable_diagnostics:
            return

        now = self.get_clock().now()
        dt = (now - self.last_diag_time).nanoseconds * 1e-9
        if dt < self.diagnostics_interval_sec:
            return
        self.last_diag_time = now

        if self.T_global_imu_first is None:
            self.T_global_imu_first = T_global_imu.copy()
            self.T_global_base_first = T_global_base.copy()
            self.T_odom_base_first = T_odom_base.copy()
            self.get_logger().info("[VIO_DIAG] Initialized diagnostic reference pose")
            return

        # Raw OpenVINS global delta
        dp_global_imu = T_global_imu[:3, 3] - self.T_global_imu_first[:3, 3]

        # Converted base pose delta in odom frame
        dp_odom_base = T_odom_base[:3, 3] - self.T_odom_base_first[:3, 3]

        # Relative rotations
        dR_global_imu = self.T_global_imu_first[:3, :3].T @ T_global_imu[:3, :3]
        dR_odom_base = self.T_odom_base_first[:3, :3].T @ T_odom_base[:3, :3]

        r_raw, p_raw, y_raw = rpy_from_rot(dR_global_imu)
        r_out, p_out, y_out = rpy_from_rot(dR_odom_base)

        # Current absolute orientation too
        r_abs, p_abs, y_abs = rpy_from_rot(T_odom_base[:3, :3])

        self.get_logger().info(
            "[VIO_DIAG]\n"
            f"  raw dp_global_imu     = "
            f"[{dp_global_imu[0]:+.3f}, {dp_global_imu[1]:+.3f}, {dp_global_imu[2]:+.3f}] m\n"
            f"  out dp_odom_base      = "
            f"[{dp_odom_base[0]:+.3f}, {dp_odom_base[1]:+.3f}, {dp_odom_base[2]:+.3f}] m\n"
            f"  raw dRPY              = "
            f"[{rad2deg(r_raw):+.1f}, {rad2deg(p_raw):+.1f}, {rad2deg(y_raw):+.1f}] deg\n"
            f"  out dRPY              = "
            f"[{rad2deg(r_out):+.1f}, {rad2deg(p_out):+.1f}, {rad2deg(y_out):+.1f}] deg\n"
            f"  out abs RPY           = "
            f"[{rad2deg(r_abs):+.1f}, {rad2deg(p_abs):+.1f}, {rad2deg(y_abs):+.1f}] deg\n"
            f"  msg frame             = {msg.header.frame_id} -> {msg.child_frame_id}\n"
            f"  adapter frame         = {self.output_frame_id} -> {self.output_child_frame_id}\n"
            f"  tf used               = {self.base_frame_id} -> {self.oak_imu_frame_id}\n"
            f"  invert_orientation    = {self.invert_openvins_orientation}"
        )


    def odom_callback(self, msg: Odometry):
        if not self.try_update_static_transform():
            return

        # OpenVINS output.
        # Important:
        #   OpenVINS may publish q_GtoI-like orientation.
        #   In that case, invert_openvins_orientation must be True.
        T_global_imu = openvins_odom_pose_to_matrix(
            msg,
            self.invert_openvins_orientation,
        )

        # Convert IMU pose to base_link pose.
        # T_global_base = T_global_imu * T_imu_base
        T_global_base = T_global_imu @ self.T_imu_base

        # Align first VIO pose to current odom -> base_link TF.
        # This prevents VIO yaw from pulling the EKF at startup.
        if not self.try_initialize_odom_global(T_global_base):
            return

        T_odom_base = self.T_odom_global @ T_global_base

        self.maybe_print_diagnostics(
            msg,
            T_global_imu,
            T_global_base,
            T_odom_base,
        )

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.output_frame_id
        out.child_frame_id = self.output_child_frame_id

        out.pose.pose.position.x = float(T_odom_base[0, 3])
        out.pose.pose.position.y = float(T_odom_base[1, 3])
        out.pose.pose.position.z = float(T_odom_base[2, 3])

        qx, qy, qz, qw = rot_to_quat(T_odom_base[:3, :3])
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw

        # ------------------------------------------------------------
        # Pose covariance transform
        # ------------------------------------------------------------
        #
        # msg.pose.covariance:
        #   covariance of OpenVINS imu pose in global frame
        #
        # out.pose.covariance:
        #   covariance of base_link pose in odom frame
        #
        # T_odom_base = T_odom_global * T_global_imu * T_imu_base
        #
        R_odom_global = self.T_odom_global[:3, :3]
        R_global_imu = T_global_imu[:3, :3]

        # Translation of base_link origin expressed in imu frame.
        # T_imu_base maps base_link coordinates into imu coordinates.
        p_base_in_imu = self.T_imu_base[:3, 3]

        out.pose.covariance = transform_pose_covariance_ros_approx(
            msg.pose.covariance,
            R_odom_global,
            R_global_imu,
            p_base_in_imu,
        )

        # ------------------------------------------------------------
        # Twist and twist covariance transform
        # ------------------------------------------------------------
        #
        # ROS Odometry convention:
        #   twist is expressed in child_frame_id.
        #
        # Input:
        #   msg.child_frame_id = imu
        #   twist at imu origin, expressed in imu frame
        #
        # Output:
        #   out.child_frame_id = base_link
        #   twist at base_link origin, expressed in base_link frame
        #
        R_base_imu = self.T_base_imu[:3, :3]

        # Translation of imu origin expressed in base_link frame.
        # T_base_imu maps imu coordinates into base_link coordinates.
        p_imu_in_base = self.T_base_imu[:3, 3]

        v_base, w_base, twist_cov_base = transform_twist_and_covariance_ros_approx(
            msg.twist.twist,
            msg.twist.covariance,
            R_base_imu,
            p_imu_in_base,
        )

        out.twist.twist.linear.x = float(v_base[0])
        out.twist.twist.linear.y = float(v_base[1])
        out.twist.twist.linear.z = float(v_base[2])

        out.twist.twist.angular.x = float(w_base[0])
        out.twist.twist.angular.y = float(w_base[1])
        out.twist.twist.angular.z = float(w_base[2])

        out.twist.covariance = twist_cov_base

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VioOdomAdapterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
