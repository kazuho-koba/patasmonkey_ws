"""globalの水平・localの高さ姿勢合成とTF整合を確認する。"""
import math
from types import SimpleNamespace
import pytest
from nav_msgs.msg import Odometry
from pm_evaluation.global_separated_composer import GlobalSeparatedComposer
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
