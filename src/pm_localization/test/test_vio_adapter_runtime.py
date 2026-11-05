"""実際のnode callbackを通し、初回整列・入力検査・保護状態を確認する。"""
import json
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from std_msgs.msg import String
from pm_localization.vio_odom_adapter_node import (
    VioOdomAdapterNode, quat_to_rot, rot_to_quat, so3_exp,
)
from pm_localization.vio_rigid_transform import transform_pose, transform_twist

C = np.array([[0., 0., -1.], [0., -1., 0.], [-1., 0., 0.]])
T = np.array([.11, -.06, .28])


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture(scope='module', autouse=True)
def context():
    rclpy.init(args=[])
    yield
    rclpy.shutdown()


@pytest.fixture
def make_node(monkeypatch):
    nodes = []
    def make(**params):
        node = VioOdomAdapterNode(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        node.pub = Capture()
        node.diagnostics = Capture()
        fixed = TransformStamped()
        fixed.transform.translation.x, fixed.transform.translation.y, fixed.transform.translation.z = map(float, T)
        fixed.transform.rotation.w = 1.
        monkeypatch.setattr(node.tf_buffer, 'lookup_transform', lambda *args: fixed)
        nodes.append(node)
        return node, fixed
    yield make
    for node in nodes:
        node.destroy_node()


def message(stamp=10., rotation=None):
    msg = Odometry()
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int(round((stamp-int(stamp))*1e9))
    msg.header.frame_id, msg.child_frame_id = 'global', 'imu'
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = 2., 3., 1.
    # 車体が水平に近いとき、raw IMUには取付回転が含まれる。
    body = so3_exp(np.array([.05, -.1, .7])) if rotation is None else rotation
    q = rot_to_quat(body @ C)
    msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q
    msg.pose.covariance = (np.diag([.02, .03, .04, .001, .003, .005])).reshape(-1).tolist()
    msg.twist.covariance = (np.eye(6)*.01).reshape(-1).tolist()
    return msg


def reference(stamp=10.):
    msg = message(stamp)
    msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = 5., -2., .1
    q = rot_to_quat(so3_exp(np.array([0., 0., -.4])))
    msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q
    return msg


def test_waits_for_ready_and_matching_reference(make_node):
    node, _ = make_node(require_heading_ready=True)
    node.odom_callback(message(9.))
    assert not node.pub.messages and node.A is None
    node.reference_callback(reference())
    node.heading_callback(String(data=json.dumps({'state': 'READY'})))
    node.odom_callback(message())
    assert len(node.pub.messages) == 1
    output = node.pub.messages[0]
    assert output.header.frame_id == 'odom' and output.child_frame_id == 'base_link'
    assert np.allclose([output.pose.pose.position.x, output.pose.pose.position.y, output.pose.pose.position.z], [5., -2., .1])
    r = quat_to_rot(output.pose.pose.orientation)
    assert np.isclose(np.arctan2(r[1, 0], r[0, 0]), -.4)
    # 固定の整列変換を実際のcallbackでも使い、候補の共分散と一致する。
    source = message()
    expected = transform_pose([2., 3., 1.], quat_to_rot(source.pose.pose.orientation),
        source.pose.covariance, C, T, node.A, node.b)[2]
    assert np.allclose(np.asarray(output.pose.covariance).reshape(6, 6), expected)
    assert np.max(np.diag(expected)[3:]) < .05  # 旧式の一律下限を加えない。


def test_timestamp_match_required(make_node):
    node, _ = make_node()
    node.reference_callback(reference(3.))
    node.odom_callback(message(10.))
    assert not node.pub.messages and node.A is None


def test_effective_mount_is_not_multiplied_twice(make_node):
    node, fixed = make_node(align_initial_to_tf=False, output_frame_id='vio_odom')
    q = rot_to_quat(C)
    fixed.transform.rotation.x, fixed.transform.rotation.y, fixed.transform.rotation.z, fixed.transform.rotation.w = q
    node.odom_callback(message())
    assert np.allclose(node.T_base_imu[:3, :3], C)
    assert len(node.pub.messages) == 1


def test_pure_rotation_does_not_invent_linear_velocity(make_node):
    node, _ = make_node(align_initial_to_tf=False, output_frame_id='vio_odom')
    omega = np.array([.7, -.3, .4])
    source = message()
    v, w = C.T @ np.cross(omega, T), C.T @ omega
    source.twist.twist.linear.x, source.twist.twist.linear.y, source.twist.twist.linear.z = map(float, v)
    source.twist.twist.angular.x, source.twist.twist.angular.y, source.twist.twist.angular.z = map(float, w)
    node.odom_callback(source)
    output = node.pub.messages[0]
    actual = output.twist.twist.linear
    assert np.allclose([actual.x, actual.y, actual.z], 0.)
    expected = transform_twist(np.r_[v, w], source.twist.covariance, C, T)[1]
    assert np.allclose(np.asarray(output.twist.covariance).reshape(6, 6), expected)


def test_bad_first_covariance_does_not_establish_alignment(make_node):
    node, _ = make_node(align_initial_to_tf=False, output_frame_id='vio_odom')
    source = message()
    source.pose.covariance[0] = -1.
    node.odom_callback(source)
    assert node.A is None and node.rejected == 1
    node.odom_callback(message(11.))
    assert len(node.pub.messages) == 1
    source = message(12.)
    source.twist.covariance[1] = .1
    node.odom_callback(source)
    assert node.rejected == 2 and len(node.pub.messages) == 1


def test_clock_rewind_requires_explicit_restart(make_node):
    node, _ = make_node(align_initial_to_tf=False, output_frame_id='vio_odom')
    node.odom_callback(message(10.))
    node.odom_callback(message(10.))  # 重複は捨てるが保護latchしない。
    assert not node.reset_required
    node.odom_callback(message(2.))
    node.odom_callback(message(20.))
    assert node.reset_required and len(node.pub.messages) == 1


def test_heading_reset_does_not_silently_realign(make_node):
    node, _ = make_node(require_heading_ready=True)
    node.reference_callback(reference())
    node.heading_callback(String(data='{"state":"READY"}'))
    node.odom_callback(message())
    a = node.A.copy()
    node.heading_callback(String(data='{"state":"WAITING"}'))
    node.odom_callback(message(11.))
    assert node.reset_required and np.allclose(node.A, a)
    assert len(node.pub.messages) == 1


def test_unreferenced_output_cannot_masquerade_as_odom(make_node):
    node, _ = make_node(align_initial_to_tf=False)
    node.odom_callback(message())
    assert not node.pub.messages and node.rejected == 1


def test_diagnostics_include_counters_and_covariance_scope(make_node):
    node, _ = make_node(align_initial_to_tf=False, output_frame_id='vio_odom')
    node.odom_callback(message())
    node.publish_diagnostics()
    values = {v.key: v.value for v in node.diagnostics.messages[-1].status[0].values}
    assert values['forwarded'] == '1' and values['received'] == '1'
    assert 'initial alignment' in values['covariance_condition']
