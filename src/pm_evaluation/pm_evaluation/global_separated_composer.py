"""評価用：global水平EKFにlocalの姿勢・高さを合成し、map→odom TFを作る。"""
import copy
import math
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from pm_localization.local_odometry_composer_node import (
    quaternion_from_rpy, rpy_from_quaternion)


class GlobalSeparatedComposer(Node):
    """水平map状態と時刻の近いlocal姿勢・高さから全姿勢を組み立てる。

    global EKFのpublish_tfを止め、このnodeだけがmap→odomを発行する。
    local zは地球絶対高度ではなく、VIO高さgate閉鎖後は保持値となる。
    """
    def __init__(self):
        super().__init__('global_separated_composer')
        self.local = None
        self.pub = self.create_publisher(Odometry, '/odometry/global', 20)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odometry/local', self.local_callback, 50)
        self.create_subscription(Odometry, '/odometry/global_horizontal', self.global_callback, 20)

    def local_callback(self, msg):
        self.local = msg

    def global_callback(self, horizontal):
        local = self.local
        if local is None:
            return
        t = horizontal.header.stamp.sec+horizontal.header.stamp.nanosec*1e-9
        lt = local.header.stamp.sec+local.header.stamp.nanosec*1e-9
        # 時刻が大きくずれた状態で不整合なTFを作らない。
        if abs(t-lt) > .2:
            return
        roll,pitch,local_yaw = rpy_from_quaternion(local.pose.pose.orientation)
        _,_,global_yaw = rpy_from_quaternion(horizontal.pose.pose.orientation)
        output = copy.deepcopy(horizontal)
        output.header.frame_id = 'map'
        output.child_frame_id = 'base_link'
        output.pose.pose.position.z = local.pose.pose.position.z
        q = quaternion_from_rpy(roll,pitch,global_yaw)
        o = output.pose.pose.orientation
        o.x,o.y,o.z,o.w = q
        for idx in (14,21,28):
            output.pose.covariance[idx] = local.pose.covariance[idx]
            output.twist.covariance[idx] = local.twist.covariance[idx]
        output.twist.twist.linear.z = local.twist.twist.linear.z
        output.twist.twist.angular.x = local.twist.twist.angular.x
        output.twist.twist.angular.y = local.twist.twist.angular.y
        self.pub.publish(output)
        # 両poseでroll/pitchとzが同じため、map→odomは水平回転・平行移動。
        # T_map_odom = T_map_base * inverse(T_odom_base)を同時刻近似で算出する。
        angle = math.atan2(math.sin(global_yaw-local_yaw),math.cos(global_yaw-local_yaw))
        c,s = math.cos(angle),math.sin(angle)
        lp = local.pose.pose.position
        gp = output.pose.pose.position
        transform = TransformStamped()
        transform.header.stamp = horizontal.header.stamp
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = gp.x-(c*lp.x-s*lp.y)
        transform.transform.translation.y = gp.y-(s*lp.x+c*lp.y)
        transform.transform.translation.z = gp.z-lp.z
        transform.transform.rotation.z = math.sin(angle/2)
        transform.transform.rotation.w = math.cos(angle/2)
        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node=GlobalSeparatedComposer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
