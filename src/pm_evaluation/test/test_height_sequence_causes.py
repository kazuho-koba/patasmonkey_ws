"""原因重複と既存100丸め境界を合成入力で確認する。"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from diagnose_height_candidate_sequence import classify


def test_cause_counts_include_overlap_and_unknown():
    result = classify([[20., .04, 0., np.nan], [0., 0., .07, .08],
                       [np.nan]*4, [0., 0., 0., 0.]], np.array([20., .03, .07, .08]))
    assert result['black_cells'] == 2 and result['known_cells'] == 3
    assert all(result[name+'_black_cells'] == 1 for name in ('slope','roughness','step','obstacle'))


def test_rounding_matches_black_map_not_strict_threshold():
    result = classify([[19.92, 0., 0., 0.], [19.8, 0., 0., 0.]], np.array([20., .03, .07, .08]))
    assert result['slope_black_cells'] == 1
