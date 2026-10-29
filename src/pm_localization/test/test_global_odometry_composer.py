"""globalの水平・localの高さ姿勢合成とTF整合を確認する。"""
import math
from types import SimpleNamespace
import pytest
from nav_msgs.msg import Odometry
from pm_localization.global_odometry_composer_node import GlobalSeparatedComposer
from pm_localization.local_odometry_composer_node import quaternion_from_rpy, rpy_from_quaternion


def make_odom(t, x,y,z,roll,pitch,yaw):
    msg=Odometry()
    msg.header.stamp.sec=t
    msg.pose.pose.position.x=x;msg.pose.pose.position.y=y;msg.pose.pose.position.z=z
    q=quaternion_from_rpy(roll,pitch,yaw)
    o=msg.pose.pose.orientation;o.x,o.y,o.z,o.w=q
    return msg


def test_composes_pose_and_tf():
    output=[]; transforms=[]
    local=make_odom(100,2.,3.,4.,.1,.2,.3)
    horizontal=make_odom(100,10.,20.,0.,0.,0.,.8)
    stub=SimpleNamespace(local=local,pub=SimpleNamespace(publish=output.append),
                         broadcaster=SimpleNamespace(sendTransform=transforms.append))
    GlobalSeparatedComposer.global_callback(stub,horizontal)
    assert len(output)==len(transforms)==1
    msg,tf=output[0],transforms[0]
    assert msg.pose.pose.position.z==4.
    assert rpy_from_quaternion(msg.pose.pose.orientation)==pytest.approx((.1,.2,.8))
    c,s=math.cos(.5),math.sin(.5)
    assert tf.transform.translation.x+c*2-s*3==pytest.approx(10.)
    assert tf.transform.translation.y+s*2+c*3==pytest.approx(20.)
    assert tf.transform.translation.z==0.
    # 時刻の古いlocalを使ったglobal出力やTFを出さない。
    horizontal.header.stamp.sec=101
    GlobalSeparatedComposer.global_callback(stub,horizontal)
    assert len(output)==len(transforms)==1


def test_covariance_cross_terms_do_not_mix_independent_estimators():
    """独立推定の高さ・傾斜と水平の相関を持ち越さない。"""
    output=[]
    local=make_odom(100,2.,3.,4.,.1,.2,.3)
    horizontal=make_odom(100,10.,20.,0.,0.,0.,.8)
    horizontal.pose.covariance[2]=42.0
    horizontal.pose.covariance[12]=42.0
    horizontal.pose.covariance[7]=3.0
    local.pose.covariance[14]=123.0
    stub=SimpleNamespace(local=local,pub=SimpleNamespace(publish=output.append),
                         broadcaster=SimpleNamespace(sendTransform=lambda _: None))
    GlobalSeparatedComposer.global_callback(stub,horizontal)
    assert output[0].pose.covariance[2] == 0.0
    assert output[0].pose.covariance[12] == 0.0
    assert output[0].pose.covariance[7] == 3.0
    assert output[0].pose.covariance[14] == 123.0


def test_relative_height_can_change_without_changing_horizontal_solution():
    """段差の相対zをlocalから受け、global x/yと姿勢を保つ。"""
    output=[]
    local=make_odom(100,0.,0.,0.,.1,.2,.3)
    horizontal=make_odom(100,10.,20.,0.,0.,0.,.8)
    stub=SimpleNamespace(local=local,pub=SimpleNamespace(publish=output.append),
                         broadcaster=SimpleNamespace(sendTransform=lambda _: None))
    GlobalSeparatedComposer.global_callback(stub,horizontal)
    local.pose.pose.position.z=1.25
    GlobalSeparatedComposer.global_callback(stub,horizontal)
    assert [m.pose.pose.position.z for m in output] == [0.0, 1.25]
    assert all(m.pose.pose.position.x == 10.0 and m.pose.pose.position.y == 20.0 for m in output)
    assert rpy_from_quaternion(output[-1].pose.pose.orientation) == pytest.approx((.1,.2,.8))
