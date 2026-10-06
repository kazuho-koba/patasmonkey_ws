"""合成点列で疎な危険・縦範囲・overflow・unknownの保持を確認する。"""

import numpy as np
import pytest

from pm_evaluation.height_candidates import CandidateOptions, extract_candidates, odom_planes, plane_reasons


def extract(z, options=None, valid=True, xy=None):
    """単一セルの原点平面を診断基準にする。正解groundはテスト側が与える。"""
    points = np.column_stack((np.full(len(z), .5), np.full(len(z), .5), z))
    if xy is not None:
        points[:, :2] = xy
    return extract_candidates(points, np.column_stack((np.arange(len(z)), np.zeros(len(z)))),
                              (0, 0), (1, 1), 1., np.zeros((1, 1, 3)),
                              [[valid]], .08, options or CandidateOptions())


def test_ground_and_one_high_pixel():
    rows, layers = extract([0, .01, .005, .2])
    assert [r['kind'] for r in rows] == ['ground_provisional', 'sparse']
    assert rows[1]['count'] == 1 and rows[1]['min_u'] == 3
    assert layers['above_plane_hit'][0, 0] == 1
    assert rows[1]['existence_probability'] is None


def test_wall_is_broad_not_a_mean_plane():
    rows, layers = extract(np.arange(0, 1., .04))
    assert len(rows) == 1 and rows[0]['kind'] == 'broad'
    assert rows[0]['z_hi_m'] > .9
    assert layers['ground_provisional'][0, 0] == 0
    assert layers['above_plane_hit'][0, 0] == 1


def test_ceiling_separated_and_vehicle_hit_not_inferred_from_gap():
    rows, layers = extract([0, .01, 1.5, 1.51], CandidateOptions(vehicle_height_m=.5))
    assert len(rows) == 2
    assert layers['above_plane_hit'][0, 0] == 1
    assert layers['body_height_hit'][0, 0] == 0
    assert not any(row['free_evidence'] for row in rows)


def test_sparse_branch_without_plane_is_not_discarded():
    rows, layers = extract([.3], valid=False)
    assert rows[0]['kind'] == 'sparse'
    assert rows[0]['min_residual_m'] is None
    assert np.isnan(layers['above_plane_hit'][0, 0])


def test_overflow_does_not_remove_high_hit():
    rows, layers = extract([0, .06, .12, .3], CandidateOptions(max_object_candidates=1))
    assert len(rows) == 2  # ground仮説1＋object枠1
    assert layers['overflow_count'][0, 0] == 2
    assert layers['above_plane_hit'][0, 0] == 1
    assert layers['above_plane_witness_index'][0, 0] == 2  # 枠外の.12mも根拠を保存


def test_outside_nan_and_empty_are_unknown():
    rows, layers = extract([.1, np.nan], xy=[[2, .5], [.5, .5]])
    assert rows == [] and layers['point_count'][0, 0] == 0
    assert np.isnan(layers['body_height_hit'][0, 0])
    rows, _ = extract([])
    assert rows == []


def test_relative_plane_offset_is_subtracted():
    baseline = dict(plane_a=np.array([[.1]]), plane_b=np.array([[.2]]),
                    plane_c=np.array([[.35]]))
    planes = odom_planes(baseline, .3)
    np.testing.assert_allclose(planes, [[[.1, .2, .05]]])


def test_plane_reason_bits_keep_overlapping_gates():
    shape = (1, 5)
    baseline = dict(relative_elevation=np.zeros(shape), age_seconds=np.zeros(shape),
                    support_count=np.array([[3, 5, 5, 5, 5]]),
                    plane_a=np.array([[np.nan, 0, 0, np.nan, 0]]),
                    plane_b=np.zeros(shape), plane_c=np.zeros(shape),
                    slope_deg=np.array([[np.nan, 20., 25., np.nan, 10.]]),
                    roughness=np.array([[np.nan, 0., .04, np.nan, .01]]))
    params = dict(feature_max_observation_age=3, feature_min_neighbors=5,
                  hazard_slope_limit_deg=20., hazard_roughness_limit=.03)
    result = plane_reasons(baseline, params)
    np.testing.assert_array_equal(result['plane_reason_bits'], [[2, 8, 24, 4, 0]])
    np.testing.assert_array_equal(result['plane_usable'], [[False, False, False, False, True]])


@pytest.mark.parametrize('kwargs', [{'gap_m': 0}, {'gap_m': np.nan},
                                   {'max_object_candidates': 0}, {'max_object_candidates': 1.5},
                                   {'vehicle_height_m': .0}, {'safety_margin_m': -1}])
def test_invalid_options(kwargs):
    with pytest.raises(ValueError):
        CandidateOptions(**kwargs)
