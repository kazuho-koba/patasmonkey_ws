"""短期高さ推定の因果性、窓、中央値と履歴不整合を検証する。"""
import numpy as np
import pytest

from pm_evaluation.ground_temporal import GroundHistory, seam_diagnostics, features


def test_windows_and_median():
    history=GroundHistory()
    for stamp,z in enumerate((0.,0.,.09)):
        result=history.update(7,stamp,z)
    assert result['mean_N3'][0]==pytest.approx(.03)
    assert result['median_N3'][0]==0.
    assert result['ground_N1'][0]==.09
    assert len(result['mean_N5'][1])==3
    # 別trackは独立。未観測はupdateを呼ばず、0で窓を進めない。
    assert history.update(8,4,.2)['mean_N5'][0]==.2
    assert len(history.histories[7])==3
    with pytest.raises(ValueError):history.update(7,2,.1)


def test_seam_without_assuming_true_bias_known():
    heights=np.array([[0.,.04]],dtype=float);reference=np.array([[.04,.04]])
    stamps=np.full((5,1,2),-1,dtype=np.int64)
    stamps[-3:,0,0]=[0,1,4];stamps[-3:,0,1]=[2,3,4]
    result=seam_diagnostics(heights,reference,stamps,4)
    assert result['neighbor_gap_m'][0]==.04
    assert result['gap_increase_vs_N1_m'][0]==.04
    assert result['different_history'][0]


def test_clean_step_is_not_spatially_smoothed():
    params=dict(resolution=.1,feature_max_observation_age=3.,feature_neighborhood_radius_cells=1,
                feature_min_neighbors=5,hazard_slope_limit_deg=20.,hazard_roughness_limit=.03,
                hazard_step_limit=.07,step_min_side_neighbors=2)
    history=GroundHistory();grid=np.zeros((9,9));grid[:,4:]=.1
    for stamp in range(5):
        estimate=np.zeros_like(grid)
        for yy,xx in np.ndindex(grid.shape):
            estimate[yy,xx]=history.update(yy*9+xx,stamp,grid[yy,xx])['mean_N5'][0]
    assert np.allclose(estimate,grid)
    f=features(estimate,params,0.)
    assert np.isfinite(f['slope_deg'][4,4])
    assert f['slope_deg'][4,4]>20.


def test_ground_gate_and_step_cue_are_not_physical_jump_height():
    from pm_evaluation.height_candidates import plane_reasons
    params=dict(resolution=.1,feature_max_observation_age=3.,feature_neighborhood_radius_cells=1,
                feature_min_neighbors=5,hazard_slope_limit_deg=20.,hazard_roughness_limit=.03,
                hazard_step_limit=.07,step_min_side_neighbors=2)
    grid=np.zeros((9,9));grid[:,4:]=.1
    f=features(grid,params,0.)
    baseline=dict(f,relative_elevation=grid,age_seconds=np.zeros_like(grid))
    gate=plane_reasons(baseline,params)
    assert not gate['plane_usable'][4,4]  # 平滑化以前にground基準から落ちる。
    assert f['step_height'][4,4]==pytest.approx(.05,abs=1e-6)
    grid[:,4:]=.07
    assert features(grid,params,0.)['step_height'][4,4]==pytest.approx(.035,abs=1e-6)
