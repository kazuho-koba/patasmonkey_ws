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


def test_accepted_ground_age_ignores_rejected_high_candidate():
    grid = RollingElevationGrid(2.0, 2.0, 0.1)
    grid.recenter(0.0, 0.0)
    x = np.array([0.25])
    y = np.array([0.25])
    grid.fuse_points(x, y, np.array([0.10]), 1_000_000_000, 0.20, 0.20)
    # 20 cmより高い値はground融合されないが、一般観測時刻は更新される。
    grid.fuse_points(x, y, np.array([0.50]), 1_200_000_000, 0.20, 0.20)
    layers = grid.stage2_layers(
        0.0025, 1_300_000_000, 0.15, 8.0,
        include_accepted_ground_age=True,
    )
    valid = layers["count"] > 0
    assert np.isclose(layers["age_seconds"][valid].item(), 0.1)
    assert np.isclose(
        layers["accepted_ground_age_seconds"][valid].item(), 0.3,
        atol=1e-6,
    )

    # groundに採用される新観測が来ると、その時刻だけは前進する。
    grid.fuse_points(x, y, np.array([0.12]), 1_250_000_000, 0.20, 0.20)
    layers = grid.stage2_layers(
        0.0025, 1_300_000_000, 0.15, 8.0,
        include_accepted_ground_age=True,
    )
    assert np.isclose(
        layers["accepted_ground_age_seconds"][valid].item(), 0.05,
        atol=1e-6,
    )


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


def test_forensic_fusion_keeps_min_max_pixel_and_before_after_ground():
    grid = RollingElevationGrid(2.0, 2.0, 0.1, forensic=True)
    grid.recenter(0.0, 0.0)
    grid.fuse_points(
        np.array([0.25, 0.25, 0.25]),
        np.array([0.25, 0.25, 0.25]),
        np.array([0.12, 0.10, 0.11]),
        1_000_000_000, 0.20, 0.20,
        pixel_u=np.array([10, 20, 30]),
        pixel_v=np.array([11, 21, 31]),
        axial_depth=np.array([1.2, 1.0, 1.1]),
        source_pose=(1.0, 2.0, 3.0, 0.1, -0.2, 0.3),
    )
    layers = grid.forensic_layers()
    cell = layers["min_depth_m"] == 1.0
    assert cell.sum() == 1
    assert layers["sample_count"][cell].item() == 3
    assert layers["min_pixel_u"][cell].item() == 20
    assert layers["min_pixel_v"][cell].item() == 21
    assert np.isclose(layers["min_world_z"][cell].item(), 0.10)
    assert np.isclose(layers["ground_before"][cell].item(), 0.0)
    assert np.isclose(layers["ground_after"][cell].item(), 0.10)
    assert layers["fusion_mode"][cell].item() == 1
    assert np.isclose(layers["base_pitch"][cell].item(), -0.2)


def test_forensic_fusion_preserves_old_and_new_accepted_ground_sources():
    grid = RollingElevationGrid(2.0, 2.0, 0.1, forensic=True)
    grid.recenter(0.0, 0.0)
    grid.fuse_points(
        np.array([0.25]), np.array([0.25]), np.array([0.30]),
        1_000_000_000, 0.20, 0.20,
        pixel_u=np.array([12]), pixel_v=np.array([34]),
        axial_depth=np.array([1.5]), source_pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    )
    # 次の撮像ではbase zが30 cm下がる。absolute candidateはfusion閾値を跨ぐが、
    # 同じ水平面のrelative値になるよう今回frameのcamera referenceを30 cm補正する。
    grid.fuse_points(
        np.array([0.25]), np.array([0.25]), np.array([0.0]),
        2_000_000_000, 0.20, 0.20,
        relative_elevation_offset=0.30,
        pixel_u=np.array([56]), pixel_v=np.array([78]),
        axial_depth=np.array([1.6]), source_pose=(0.0, 0.0, -0.30, 0.0, 0.0, 0.0),
    )
    layers = grid.forensic_layers()
    cell = layers["fusion_mode"] == 2
    assert cell.sum() == 1
    assert layers["previous_ground_input_stamp_ns"][cell].item() == 1_000_000_000
    assert np.isclose(layers["previous_ground_input_world_z"][cell].item(), 0.30)
    assert layers["previous_ground_input_pixel_u"][cell].item() == 12
    assert np.isclose(layers["previous_ground_input_base_z"][cell].item(), 0.0)
    assert layers["ground_input_stamp_ns"][cell].item() == 2_000_000_000
    assert np.isclose(layers["ground_input_world_z"][cell].item(), 0.0)
    assert layers["ground_input_pixel_u"][cell].item() == 56
    assert np.isclose(layers["ground_input_base_z"][cell].item(), -0.30)
    assert np.isclose(layers["ground_input_relative_z"][cell].item(), 0.30)
    assert np.isclose(layers["relative_after"][cell].item(), 0.30)
