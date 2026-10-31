"""Path/OdometryのROS接続とdebug出力先を、実車非接続で検証する。"""
import json
import os
import time
import pytest

rclpy=pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import String
from pm_planner.mppi_poc_node import MppiPocNode


def test_debug_only_and_invalid_frame_stops():
    if os.environ.get('ROS_DOMAIN_ID')!='213':pytest.skip('ROS_DOMAIN_ID=213で隔離して実行する')
    rclpy.init(args=[]);executor=SingleThreadedExecutor()
    node=MppiPocNode([Parameter('batch_size',value=128),Parameter('visualize',value=True)])
    helper=Node('mppi_poc_test');executor.add_node(node);executor.add_node(helper)
    path_pub=helper.create_publisher(Path,'/pm/planner/preview/reference_path',
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
    odom_pub=helper.create_publisher(Odometry,'/odometry/global',qos_profile_sensor_data)
    commands=[];statuses=[];trajectories=[]
    helper.create_subscription(Twist,'/pm/planner/mppi_poc/debug/cmd_vel',commands.append,10)
    helper.create_subscription(String,'/pm/planner/mppi_poc/debug/status',lambda msg:statuses.append(json.loads(msg.data)),10)
    helper.create_subscription(Path,'/pm/planner/mppi_poc/debug/trajectory',trajectories.append,10)
    def wait(predicate):
        end=time.monotonic()+5
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.02)
            if predicate():return
        assert predicate(),'ROS応答timeout'
    try:
        wait(lambda:path_pub.get_subscription_count()>0 and odom_pub.get_subscription_count()>0)
        path=Path();path.header.frame_id='map'
        for x in (0.,2.,5.):
            p=PoseStamped();p.pose.position.x=x;p.pose.orientation.w=1.;path.poses.append(p)
        path_pub.publish(path)
        odom=Odometry();odom.header.frame_id='map';odom.child_frame_id='base_link'
        odom.header.stamp=helper.get_clock().now().to_msg();odom.pose.pose.orientation.w=1.;odom_pub.publish(odom)
        wait(lambda:statuses and statuses[-1]['valid'] and commands and trajectories and trajectories[-1].poses)
        assert commands[-1].linear.x>=0 and trajectories[-1].header.frame_id=='map'
        names=[name for name,_ in node.get_publisher_names_and_types_by_node(node.get_name(),node.get_namespace())]
        assert '/cmd_vel' not in names and '/cmd_vel_autonomy' not in names
        odom.header.frame_id='odom';odom.header.stamp=helper.get_clock().now().to_msg();odom_pub.publish(odom)
        wait(lambda:statuses and 'frame不一致' in statuses[-1]['reason'])
        assert commands[-1].linear.x==0. and commands[-1].angular.z==0.
        wait(lambda:trajectories and not trajectories[-1].poses)
    finally:
        executor.remove_node(node);executor.remove_node(helper);node.destroy_node();helper.destroy_node()
        executor.shutdown();rclpy.shutdown()
