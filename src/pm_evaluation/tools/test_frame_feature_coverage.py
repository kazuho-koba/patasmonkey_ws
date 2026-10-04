"""A相当のcoverage試験：unknown保持・期限・future排除・接続proxyを検証する。"""
import numpy as np
from evaluate_frame_feature_coverage import FeatureMemory, describe, aggregate, footprint_cells


def test_unknown_does_not_erase_and_expiry_is_per_cue():
    memory = FeatureMemory([(0, 0)], [20, .03, .07, .08])
    memory.update(np.array([[2, .01, .02, np.nan]]), 1_000_000_000)
    memory.update(np.array([[np.nan, .02, np.nan, .1]]), 3_000_000_000)
    values = memory.query(4_000_000_000, 2)
    assert np.isnan(values[0, 0]) and np.isnan(values[0, 2])
    assert values[0, 1] == .02 and values[0, 3] == .1


def test_future_stamp_is_never_valid_and_thresholds_are_metric():
    memory = FeatureMemory([(0, 0)], [20, .03, .07, .08])
    memory.update(np.array([[2, .01, .02, .1]]), 2_000_000_000)
    assert not np.isfinite(memory.query(1_000_000_000, 3)).any()
    result = describe(memory.query(2_000_000_000, 3), memory.limits)
    assert result["black_cells"] == 1 and result["all_cue_cells"] == 1


def test_separately_seen_neighbors_are_not_joint_support():
    memory = FeatureMemory([(0, 0), (1, 0)], [20, .03, .07, .08])
    memory.update(np.array([[2, .01, .02, np.nan], [np.nan]*4]), 1)
    memory.update(np.array([[np.nan]*4, [2, .01, .02, np.nan]]), 2)
    assert not memory.edges
    memory.update(np.array([[2, .01, .02, np.nan]]*2), 3)
    assert memory.edges[(0, 1)] == 3


def test_footprint_denominator_includes_cells_outside_current_map():
    cells = footprint_cells((100, -10, -10, 0, 0, 0, 0), .1, .45, .55)
    assert len(cells) > 0 and all(x < 0 and y < 0 for x, y in cells)
    row = describe(np.full((len(cells), 4), np.nan), [20, .03, .07, .08])
    summary = aggregate([row])
    assert summary["completely_unknown"] == 1
    assert summary["unknown_cell_percent"] == 100
    assert summary["black_percent_of_known"] is None
