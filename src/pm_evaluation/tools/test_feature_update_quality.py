"""診断の閾値一致・更新遷移・黒episode起点・hook復旧を確認する。"""
import csv
import numpy as np
import diagnose_feature_update_quality as diagnostic
import evaluate_spatial_feature_coverage as evaluation


def test_transition_rounding_and_unknown():
    assert diagnostic.transition(np.nan, 20, 20) == "unknown_to_black"
    assert diagnostic.transition(0, 19.91, 20) == "safe_to_black"
    assert diagnostic.transition(20, 0, 20) == "black_to_safe"
    assert diagnostic.transition(20, 21, 20) == "black_to_black"


def test_diagnostic_preserves_update_and_episode(monkeypatch, tmp_path):
    """黒→黒の再更新でも起点を安全→黒のまま保ち、実際の保持値を変えない。"""
    params = dict(resolution=1., feature_neighborhood_radius_cells=1,
                  hazard_slope_limit_deg=20., hazard_roughness_limit=.03,
                  hazard_step_limit=.07, hazard_obstacle_height_limit=.08)
    base = dict(translation=[.5, .5, 0.], quaternion=[0., 0., 0., 1.])
    monkeypatch.setattr(evaluation, "load_mapper_trace", lambda _: (dict(parameters=params),
        {10: dict(base_to_map=base)}, []))
    monkeypatch.setattr(evaluation, "sampled_points", lambda *a, **k: np.tile([.1, .1, 1.], (4, 1)))

    def evaluated(points, p, stamp, **kwargs):
        a = dict(origin=np.array([-1., -1.]), resolution=np.array(1.))
        for name in ("hazard", "support_count", "step_support_forward", "step_support_rear",
                     "relative_elevation", "residual_rms")+evaluation.CUES:
            a[name] = np.zeros((3, 3))
        a["slope_deg"][1, 1] = 0 if stamp == 1 else 20
        a["pixel_count"] = np.zeros((3, 3)); a["pixel_count"][1, 1] = 4
        a["support_count"][:] = 9
        return a
    monkeypatch.setattr(evaluation, "evaluate_points", evaluated)
    output = tmp_path/"result"
    original_store = evaluation.SpatialFeatureStore

    def pipeline():
        output.mkdir()
        store = evaluation.SpatialFeatureStore()
        for stamp in (1, 2, 3):
            points = evaluation.sampled_points()
            arrays = evaluation.evaluate_points(points, params, stamp, heading_yaw=0.)
            values, support = evaluation.extract(arrays, [(0, 0)])
            store.update([(0, 0)], values, stamp, support)
        values = store.query([(0, 0)], 9, (10, .5, .5, 0., 0., 0., 0.))
        assert values[0, 0] == 20
    monkeypatch.setattr(evaluation, "main", pipeline)
    monkeypatch.setattr("sys.argv", ["tool", "bag", "--mapper-trace", "trace", "--output", str(output)])
    diagnostic.main()
    assert evaluation.SpatialFeatureStore is original_store
    rows = list(csv.DictReader((output/"passage_black_origins.csv").open()))
    assert len(rows) == 1
    assert rows[0]["episode_transition"] == "safe_to_black"
    assert rows[0]["episode_stamp_ns"] == "2"
    assert len(list(csv.DictReader((output/"quality_events.csv").open()))) == 12
