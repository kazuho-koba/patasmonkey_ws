"""独立デモの移動・可視化・一時停止と実車非接続を確認する。"""
import json
import os
import time

import numpy as np
import pytest

from pm_planner.mppi_poc import MppiConfig, MppiPoc


def test_candidate_sampling_does_not_change_solution():
    # 表示は乱数や最適化に影響せず、選んだ候補だけ座標を保持する。
    results = []
    for count in (0, 4):
        solver = MppiPoc(MppiConfig(batch_size=32, steps=10, candidate_sample_count=count))
        solver.set_path([(0., 0.), (5., 0.)])
        results.append(solver.step((0., 0., 0.), (0., 0.)))
    assert results[0]['candidate_trajectories'] is None
    assert results[1]['candidate_trajectories'].shape == (4, 10, 2)
    np.testing.assert_allclose(results[0]['trajectory'], results[1]['trajectory'])
    np.testing.assert_allclose(results[0]['command'], results[1]['command'])


def test_live_demo_ros():
    if os.environ.get('ROS_DOMAIN_ID') != '213':
        pytest.skip('ROS_DOMAIN_ID=213で隔離して実行する')
    rclpy = pytest.importorskip('rclpy')
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from nav_msgs.msg import Odometry, Path
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker
    from pm_planner.mppi_live_demo import MppiLiveDemo

    rclpy.init(args=[])
    executor = SingleThreadedExecutor()
    node = MppiLiveDemo([Parameter('batch_size', value=32),
                         Parameter('candidate_sample_count', value=4),
                         Parameter('time_scale', value=2.0)])
    helper = Node('mppi_live_demo_test')
    executor.add_node(node)
    executor.add_node(helper)
    states, odoms, candidates, predictions = [], [], [], []
    helper.create_subscription(String, '/pm/planner/mppi_demo/status',
                               lambda msg: states.append(json.loads(msg.data)), 10)
    helper.create_subscription(Odometry, '/pm/planner/mppi_demo/odometry', odoms.append, 10)
    helper.create_subscription(Marker, '/pm/planner/mppi_demo/candidates', candidates.append, 10)
    helper.create_subscription(Path, '/pm/planner/mppi_demo/prediction', predictions.append, 10)

    def wait(predicate):
        end = time.monotonic() + 5
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.02)
            if predicate():
                return
        assert predicate(), 'ROS応答timeout'

    try:
        wait(lambda: states and states[-1]['sim_time_s'] >= .5 and odoms and
             candidates and candidates[-1].points and predictions and predictions[-1].poses)
        assert states[-1]['drive_output'] is False
        assert states[-1]['environment_checked'] is False
        assert len(node.reference.samples) > 10
        assert node.config.min_radius == pytest.approx(.3)
        assert node.reference.limits.radius == pytest.approx(.4)
        assert node.config.yaw_response_gain == pytest.approx(1.)
        assert odoms[-1].header.frame_id == 'planner_preview'
        assert np.hypot(node.x-node.reference.samples[0][0], node.y-node.reference.samples[0][1]) > .001
        topics = [name for name, _ in node.get_publisher_names_and_types_by_node(node.get_name(), node.get_namespace())]
        assert '/cmd_vel' not in topics and '/cmd_vel_autonomy' not in topics and '/clock' not in topics
        from std_srvs.srv import Trigger
        node.pause(Trigger.Request(), Trigger.Response())
        frozen = node.sim_time
        wait(lambda: states[-1]['state'] == 'PAUSED')
        assert node.sim_time == frozen
        node.resume(Trigger.Request(), Trigger.Response())
        wait(lambda: node.sim_time > frozen)
        node.reset(Trigger.Request(), Trigger.Response())
        assert node.sim_time == 0.0
    finally:
        executor.remove_node(node)
        executor.remove_node(helper)
        node.destroy_node()
        helper.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
