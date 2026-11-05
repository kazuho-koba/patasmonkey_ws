"""地球方位へ初期化済みlocalにglobal補正TFの回転を二重適用しないことを確認する。"""
import json
import math
from pathlib import Path
import pytest
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from pm_evaluation.localization_heading import initial_heading


@pytest.fixture
def config():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('pm_config'))/'config'


def sample(yaw_deg=-88.5377,seed_deg=-88.491):
    local=Odometry();local.header.frame_id='odom';local.header.stamp.sec=10
    angle=math.radians(yaw_deg);local.pose.pose.orientation.z=math.sin(angle/2);local.pose.pose.orientation.w=math.cos(angle/2)
    status=String();status.data=json.dumps(dict(state='READY',seed_yaw_rad=math.radians(seed_deg)))
    tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom';tf.header.stamp.sec=10
    tf.transform.rotation.z=math.sin(math.radians(-70.5304)/2);tf.transform.rotation.w=math.cos(math.radians(-70.5304)/2)
    data={'/odometry/local':[(10_000_000_000,local)],'/localization/initialization/status':[(10_200_000_000,status)],'/tf':[(10_000_000_000,TFMessage(transforms=[tf]))]}
    snapshot={'snapshots':{'/heading_initializer_node':{'values':{'offset_deg':-12.9701}}}}
    return data,snapshot


def test_ready_local_yaw_does_not_include_map_odom(config):
    data,snapshot=sample();yaw,offset,note=initial_heading(data,snapshot,config)
    assert math.degrees(yaw)==pytest.approx(-88.5377)
    assert offset==-12.9701 and not note['map_to_odom_applied']
    assert note['initialization_state']=='READY'


def test_current_pose_is_used_after_vehicle_turns_since_startup(config):
    data,snapshot=sample(yaw_deg=35.,seed_deg=-88.491)
    yaw,_,_=initial_heading(data,snapshot,config)
    assert math.degrees(yaw)==pytest.approx(35.)


def test_missing_ready_requires_explicit_heading(config):
    data,snapshot=sample();data.pop('/localization/initialization/status')
    with pytest.raises(ValueError,match='READY'):initial_heading(data,snapshot,config)
    yaw,_,note=initial_heading(data,snapshot,config,forced=.7)
    assert yaw==.7 and note['method']=='--initial-yaw-deg指定'


def test_pending_at_bag_start_is_not_replaced_with_later_ready(config):
    data,snapshot=sample();pending=String();pending.data=json.dumps(dict(state='WAITING'))
    data['/localization/initialization/status'].insert(0,(10_000_000_000,pending))
    with pytest.raises(ValueError,match='READY'):initial_heading(data,snapshot,config)


def test_stale_ready_is_not_sufficient(config):
    data,snapshot=sample();row=data['/localization/initialization/status'][0]
    data['/localization/initialization/status']=[(1,row[1])]
    with pytest.raises(ValueError,match='READY'):initial_heading(data,snapshot,config)


def test_both_cli_variants_share_the_same_heading_rule():
    from pm_evaluation.cli.analyze_localization_batch import heading as old
    from pm_evaluation.cli.analyze_localization_batch_twopass import heading as new
    assert old is new is initial_heading


@pytest.mark.parametrize('offset', [-12.9701, 0., 20.])
def test_magnetic_off_matches_runtime_bearing_convention(offset):
    """実機の北基準bearing→ENU変換からON/OFFの符号を独立に検証する。"""
    from pm_evaluation.localization_heading import magnetic_seed_yaw
    from pm_localization.heading_initializer_node import magnetic_yaw
    field = (40., 15., 8.)
    yaw_on = magnetic_yaw(field, .1, -.2, offset, 3.)
    yaw_off = magnetic_yaw(field, .1, -.2, 0., 3.)
    assert magnetic_seed_yaw(yaw_on, offset, 'on') == yaw_on
    delta = magnetic_seed_yaw(yaw_on, offset, 'off') - yaw_off
    assert math.atan2(math.sin(delta), math.cos(delta)) == pytest.approx(0.)


@pytest.mark.parametrize('bad', [float('nan'), float('inf')])
def test_nonfinite_magnetic_settings_are_rejected(config, bad):
    from pm_evaluation.localization_heading import magnetic_seed_yaw
    data, snapshot = sample()
    snapshot['snapshots']['/heading_initializer_node']['values']['offset_deg'] = bad
    with pytest.raises(ValueError, match='非有限'):
        initial_heading(data, snapshot, config)
    with pytest.raises(ValueError, match='非有限'):
        magnetic_seed_yaw(0., bad, 'off')
    data, snapshot = sample()
    with pytest.raises(ValueError, match='非有限'):
        initial_heading(data, snapshot, config, forced=bad)
