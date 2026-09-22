import numpy as np

from pm_perception.rolling_elevation_grid import RollingElevationGrid


def test_repeated_observations_are_fused():
    grid = RollingElevationGrid(2.0, 2.0, 0.1)
    grid.recenter(0.0, 0.0)
    for index, height in enumerate((0.10, 0.14, 0.12)):
        grid.fuse_points(
            np.array([0.25]), np.array([0.25]), np.array([height]),
            index + 1, 0.2, 0.2
        )
    elevation, variance, count, last = grid.logical_layers(0.0025)
    observed = count > 0
    assert count[observed].item() == 3
    assert np.isclose(elevation[observed].item(), 0.12)
    assert variance[observed].item() > 0.0025
    assert last[observed].item() == 3


def test_odom_cell_persists_when_window_moves_but_stays_in_bounds():
    grid = RollingElevationGrid(2.0, 2.0, 0.1)
    grid.recenter(0.0, 0.0)
    grid.fuse_points(
        np.array([0.4]), np.array([0.0]), np.array([0.2]), 1, 0.2, 0.2
    )
    grid.recenter(0.3, 0.0)
    elevation, _, count, _ = grid.logical_layers(0.0025)
    assert np.isclose(elevation[count > 0].item(), 0.2)


def test_reused_ring_slot_is_cleared_after_leaving_map():
    grid = RollingElevationGrid(1.0, 1.0, 0.1)
    grid.recenter(0.0, 0.0)
    grid.fuse_points(
        np.array([0.0]), np.array([0.0]), np.array([0.1]), 1, 0.2, 0.2
    )
    grid.recenter(1.0, 0.0)
    grid.fuse_points(
        np.array([1.0]), np.array([0.0]), np.array([0.8]), 2, 0.2, 0.2
    )
    elevation, _, count, _ = grid.logical_layers(0.0025)
    assert count[count > 0].item() == 1
    assert np.isclose(elevation[count > 0].item(), 0.8)


def test_relative_layer_uses_camera_ground_reference_at_each_stamp():
    grid = RollingElevationGrid(2.0, 2.0, 0.1)
    grid.recenter(0.0, 0.0)
    # A point at odom z=1.10 observed by a camera at z=1.50 with a nominal
    # 0.40 m camera-ground distance represents locally level ground (0 m).
    grid.fuse_points(
        np.array([0.0]), np.array([0.0]), np.array([1.10]),
        1_000_000_000, 0.2, 0.2,
        observation_variance=np.array([0.0025]),
        relative_elevation_offset=-1.10,
    )
    layers = grid.stage2_layers(0.0025, 1_000_000_000, 0.15, 8.0)
    valid = layers["count"] > 0
    assert np.isclose(layers["relative_elevation"][valid].item(), 0.0)


def test_farther_observation_has_less_fusion_weight_and_stale_weight_decays():
    grid = RollingElevationGrid(2.0, 2.0, 0.1)
    grid.recenter(0.0, 0.0)
    point = (np.array([0.0]), np.array([0.0]), np.array([0.10]))
    grid.fuse_points(
        *point, 1_000_000_000, 0.2, 0.2,
        observation_variance=np.array([0.01]), observation_decay_time=2.0,
    )
    first_weight = grid.elevation_weight.max()
    grid.fuse_points(
        *point, 5_000_000_000, 0.2, 0.2,
        observation_variance=np.array([0.01]), observation_decay_time=2.0,
    )
    # Four seconds old evidence is downweighted before the new measurement.
    assert grid.elevation_weight.max() < first_weight * 2.0
