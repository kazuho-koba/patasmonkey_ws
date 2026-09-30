"""mapper負荷比較parameterとStage 2 debug切替を検査する。"""

from collections import deque
from types import SimpleNamespace

import numpy as np

import pm_perception.depth_elevation_mapper_node as mapper_module
from pm_perception.depth_elevation_mapper_node import (
    DepthElevationMapper,
    retry_period_from_rate_hz,
)


class _Publisher:
    """publishされたdebug message数を数えるROS publisher stub。"""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_retry_rate_is_converted_to_timer_period_and_validated():
    """100/30/20 Hz比較値が意図した秒周期になり、不正値を拒否する。"""
    assert np.isclose(retry_period_from_rate_hz(100.0), 0.010)
    assert np.isclose(retry_period_from_rate_hz(30.0), 1.0 / 30.0)
    assert np.isclose(retry_period_from_rate_hz(20.0), 0.050)

    for invalid_rate in (0.0, -1.0, float("nan"), float("inf")):
        try:
            retry_period_from_rate_hz(invalid_rate)
        except ValueError:
            continue
        raise AssertionError("invalid retry rate was accepted")


def _debug_publish_state(stage2_enabled):
    """publish_debug()が使う状態だけを作り、ROS graphを起動せずに試験する。"""
    elevation = np.array([[0.1]], dtype=np.float32)
    count = np.array([[1]], dtype=np.uint32)
    scalar = np.array([[0.0]], dtype=np.float32)
    layers = {
        "elevation": elevation,
        "count": count,
        "relative_elevation": scalar,
        "variance": scalar,
        "age_seconds": scalar,
        "obstacle_height": np.array([[np.nan]], dtype=np.float32),
    }
    publishers = {
        name: _Publisher()
        for name in (
            "occupancy_publisher", "relative_elevation_publisher",
            "variance_publisher", "count_publisher", "age_publisher",
            "obstacle_publisher", "slope_publisher", "roughness_publisher",
            "step_height_publisher", "hazard_publisher",
            "hazard_cause_publisher", "hazard_cause_marker_publisher",
            "pointcloud_publisher",
        )
    }
    state = SimpleNamespace(
        grid=SimpleNamespace(
            resolution=0.10,
            stage2_layers=lambda *args, **kwargs: layers,
        ),
        measurement_variance=0.01,
        obstacle_confidence_min=0.15,
        observation_decay_time=8.0,
        feature_max_accepted_ground_age=0.0,
        publish_debug_occupancy=True,
        publish_stage2_debug_layers=stage2_enabled,
        publish_stage3_debug_layers=True,
        publish_debug_pointcloud=False,
        publish_hazard_cause_markers=False,
        feature_max_observation_age=3.0,
        feature_neighborhood_radius_cells=1,
        feature_min_neighbors=5,
        hazard_slope_limit_deg=20.0,
        hazard_roughness_limit=0.03,
        hazard_step_limit=0.07,
        hazard_obstacle_height_limit=0.20,
        step_min_side_neighbors=2,
        latest_heading_yaw=0.0,
        forensic_enabled=False,
        diagnostic_callback_timing=True,
        diagnostic_stage2_publish_ms=deque(maxlen=10),
        feature_times_ms=deque(maxlen=10),
        debug_elevation_min=-0.5,
        debug_elevation_max=0.5,
        debug_relative_elevation_min=-0.3,
        debug_relative_elevation_max=0.3,
        debug_variance_max=0.02,
        debug_count_saturation=10,
        debug_age_max=5.0,
        debug_obstacle_height_max=0.2,
        debug_slope_max_deg=20.0,
        debug_roughness_max=0.03,
        debug_step_height_max=0.07,
        make_debug_grid=lambda stamp, layer, valid, minimum, maximum: layer,
    )
    for name, publisher in publishers.items():
        setattr(state, name, publisher)
    return state, publishers


def test_stage2_layer_toggle_does_not_change_hazard_compute_or_publish(monkeypatch):
    """Stage 2出力だけを切り、Stage 3 hazard計算と出力を両条件で維持する。"""
    feature_calls = []
    scalar = np.array([[0.0]], dtype=np.float32)
    features = {
        "slope_deg": scalar,
        "roughness": scalar,
        "step_height": scalar,
        "hazard": scalar,
        "max_cause": np.array([[1]], dtype=np.uint8),
    }

    def fake_compute_terrain_features(*args, **kwargs):
        feature_calls.append(1)
        return features

    monkeypatch.setattr(
        mapper_module, "compute_terrain_features", fake_compute_terrain_features
    )

    states = {}
    for enabled in (True, False):
        state, publishers = _debug_publish_state(enabled)
        DepthElevationMapper.publish_debug(
            state, SimpleNamespace(sec=1, nanosec=0)
        )
        states[enabled] = (state, publishers)

    # どちらの比較条件でもStage 3を一度計算し、hazard/causeをpublishする。
    assert len(feature_calls) == 2
    for enabled, (state, publishers) in states.items():
        assert len(publishers["occupancy_publisher"].messages) == 1
        assert len(publishers["hazard_publisher"].messages) == 1
        assert len(publishers["hazard_cause_publisher"].messages) == 1
        stage2_names = (
            "relative_elevation_publisher", "variance_publisher",
            "count_publisher", "age_publisher", "obstacle_publisher",
        )
        expected_stage2_count = 1 if enabled else 0
        assert all(
            len(publishers[name].messages) == expected_stage2_count
            for name in stage2_names
        )
        assert len(state.diagnostic_stage2_publish_ms) == expected_stage2_count
