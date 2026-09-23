import numpy as np

from pm_perception.terrain_features import compute_terrain_features


def _features(values, **kwargs):
    return compute_terrain_features(
        values, np.zeros_like(values), 0.1, 1.0, 1, 5,
        20.0, 0.03, 0.10, **kwargs
    )


def test_plane_slope_is_recovered_without_roughness():
    x = np.arange(7, dtype=np.float32) * 0.1
    plane = np.tile(0.1 * x, (7, 1))
    result = _features(plane)
    assert np.isclose(result["slope_deg"][3, 3], np.degrees(np.arctan(0.1)))
    assert result["roughness"][3, 3] < 1e-5
    assert result["step_height"][3, 3] < 1e-5
    assert result["hazard"][3, 3] < 1.0


def test_step_and_obstacle_increase_preliminary_hazard():
    values = np.zeros((7, 7), dtype=np.float32)
    values[:, 4:] = 0.20
    result = _features(values, obstacle_height=np.zeros_like(values))
    # Fitting the local plane removes the smooth component; the remaining
    # peak-to-peak residual still detects this sharp 20 cm discontinuity.
    assert result["step_height"][3, 3] >= 0.09
    assert result["hazard"][3, 3] == 1.0


def test_stale_and_missing_observations_remain_unknown():
    values = np.zeros((5, 5), dtype=np.float32)
    age = np.full((5, 5), 2.0, dtype=np.float32)
    result = compute_terrain_features(
        values, age, 0.1, 1.0, 1, 5, 20.0, 0.03, 0.1
    )
    assert np.isnan(result["hazard"]).all()


def test_obstacle_cue_is_hazardous_without_slope_neighbours():
    values = np.full((3, 3), np.nan, dtype=np.float32)
    obstacle = np.full((3, 3), np.nan, dtype=np.float32)
    obstacle[1, 1] = 0.20
    result = compute_terrain_features(
        values, np.zeros_like(values), 0.1, 1.0, 1, 5,
        20.0, 0.03, 0.10, obstacle, 0.20,
    )
    assert result["hazard"][1, 1] == 1.0


def test_step_requires_observations_on_both_heading_sides():
    values = np.full((5, 5), np.nan, dtype=np.float32)
    values[:, 2:] = 0.10
    result = _features(values, step_min_side_neighbors=2)
    assert np.isnan(result["step_height"][2, 2])
