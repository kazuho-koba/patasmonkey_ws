"""短期候補平均がground／危険／unknownを混ぜないことを確認する。"""
import pytest
from pm_evaluation.height_candidate_tracks import CandidateTracks


def row(z=0., kind='sparse', hit=True):
    return dict(z_lo_m=z, z_hi_m=z+.01, z_mean_m=z+.005,
                x_lo_m=0., x_hi_m=.02, y_lo_m=0., y_hi_m=.02,
                kind=kind, high_hit=hit)


def test_three_hits_confirm_and_average():
    model = CandidateTracks(3)
    for i, z in enumerate([.1, .12, .11]):
        result = model.update((0, 0), [row(z)], i, i)[0]
        assert result['pending'] == (i < 2)
    assert result['confirmed']
    assert result['mean_z_m'] == pytest.approx(.115)


def test_missing_is_neutral_and_new_hazard_immediate():
    model = CandidateTracks(3)
    first = model.update((0, 0), [row()], 0, 0)[0]
    assert first['pending'] and not first['confirmed']
    model.update((0, 0), [], 1, 1)
    assert len(model.cells[(0, 0)][0]['history']) == 1
    result = model.update((0, 0), [row(hit=None)], 2, 2)[0]
    assert result['pending'] and result['positive_support'] == 1


def test_ground_never_matches_object_and_far_z_is_new():
    model = CandidateTracks()
    model.update((0, 0), [row()], 0, 0)
    assert not model.update((0, 0), [row(kind='ground_provisional')], 1, 1)[0]['matched']
    assert not model.update((0, 0), [row(z=.3)], 2, 2)[0]['matched']


def test_one_to_one_matching():
    model = CandidateTracks()
    model.update((0, 0), [row()], 0, 0)
    results = model.update((0, 0), [row(), row(.01)], 1, 1)
    assert sum(r['matched'] for r in results) == 1


def test_broad_does_not_absorb_thin_at_edge():
    model = CandidateTracks()
    broad = dict(row(), kind='broad', z_hi_m=1., z_mean_m=.5)
    model.update((0, 0), [broad], 0, 0)
    assert not model.update((0, 0), [row(.9)], 1, 1)[0]['matched']
    broad2 = dict(broad, z_lo_m=.8, z_hi_m=1.8, z_mean_m=1.3)
    assert not model.update((0, 0), [broad2], 2, 2)[0]['matched']


def test_window_is_recent_not_unbounded_average():
    model = CandidateTracks(3)
    for i, z in enumerate([0., .01, .02, .03]):
        result = model.update((0, 0), [row(z)], i, i)[0]
    assert result['mean_z_m'] == pytest.approx(.025)


def test_five_observations_confirm_only_at_fifth():
    model = CandidateTracks(5)
    for i in range(5):
        result = model.update((0, 0), [row(z=.01*i)], i, i)[0]
        assert result['confirmed'] == (i == 4)
        assert result['first_confirmation'] == (i == 4)
    assert result['history_size'] == 5
    assert result['mean_z_m'] == pytest.approx(.025)
    result = model.update((0, 0), [row(z=.04, hit=None)], 5, 5)[0]
    assert not result['confirmed'] and result['pending']
    assert result['positive_support'] == 4  # unknownをfalse／freeの票にはしない
