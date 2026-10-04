"""前方半円・停止中の保持・unknown非消去・同一frame組保持のテスト。"""
import numpy as np
import json
from types import SimpleNamespace
import evaluate_spatial_feature_coverage as tool
from evaluate_spatial_feature_coverage import front_cells, SpatialFeatureStore


def test_front_semicircle_is_distance_based_and_heading_aware():
    pose = (1, 0, 0, 0, 0, 0, 0)
    cells = front_cells(pose, .1, 2)
    assert cells
    assert all((x+.5)*.1 >= 0 and ((x+.5)*.1)**2+((y+.5)*.1)**2 <= 4 for x,y in cells)
    reverse = front_cells((1, 0, 0, 0, 0, 0, np.pi), .1, 2)
    assert all((x+.5)*.1 <= 0 for x,y in reverse)


def test_stopping_does_not_expire_and_future_is_not_available():
    store = SpatialFeatureStore()
    store.update([(0, 0)], np.array([[2, .01, .02, np.nan]]), 10, np.ones((1,4)))
    assert np.isfinite(store.query([(0, 0)], 10**15, None)[0,:3]).all()
    assert not np.isfinite(store.query([(0, 0)], 9, None)).any()
    store.update([(0, 0)], np.full((1,4), np.nan), 20, np.ones((1,4)))
    assert store.data[(0,0)][1][0] == 10
    assert not np.isfinite(store.query([(0,0)], 10**15, None, 3)).any()
    assert np.isfinite(store.query([(0,0)], 10**15, None)[0,:3]).all()


def test_coherent_triplet_and_unobserved_cells():
    store = SpatialFeatureStore()
    store.update([(0, 0)], np.array([[2, .01, np.nan, .1]]), 10, np.ones((1,4)))
    assert np.isnan(store.query([(0,0)], 10, None)[0,:3]).all()
    assert store.query([(0,0)], 10, None)[0,3] == .1
    assert np.isnan(store.query([(1,1)], 10, None)).all()


def test_main_passage_query_is_before_same_stamp_image(monkeypatch, tmp_path):
    """bag走査からCSV/NPZ出力までを合成入力で確認し、通過時future混入を拒否する。"""
    frames = {}
    for stamp, x in ((10, 0), (20, 1)):
        transform = dict(translation=[x, 0, 0], quaternion=[0, 0, 0, 1])
        frames[stamp] = dict(base_to_map=transform, camera_to_map=transform, intrinsics=[1]*4,
                             map_frame="odom", image_frame="camera", width=1, height=1)
    params = dict(resolution=.1, hazard_slope_limit_deg=20, hazard_roughness_limit=.03,
                  hazard_step_limit=.07, hazard_obstacle_height_limit=.08, depth_topic="depth",
                  min_depth=.4, max_depth=5, nominal_camera_height_above_ground=.4,
                  measurement_variance=.0025, depth_variance_per_meter_sq=.0004, obstacle_min_height=.03)
    monkeypatch.setattr(tool, "load_mapper_trace", lambda _: (dict(parameters=params), frames, []))
    records = [SimpleNamespace(header=SimpleNamespace(frame_id="camera", stamp=SimpleNamespace(sec=0,nanosec=t)),
                               width=1,height=1) for t in frames]
    monkeypatch.setattr(tool, "messages", lambda *args: [(None,None,SimpleNamespace(data=r)) for r in records])
    monkeypatch.setattr(tool, "deserialize_message", lambda r, cls: r)
    monkeypatch.setattr(tool, "sampled_points", lambda *args: np.array([[0,0,1.]]))
    arrays = dict(origin=np.array([-4,-4]), resolution=np.array(.1))
    for name in ("hazard",)+tool.CUES+("support_count","step_support_forward","step_support_rear","pixel_count","cell_min","cell_max"):
        arrays[name] = np.full((80,80), .01)
    monkeypatch.setattr(tool, "evaluate_points", lambda *args, **kwargs: arrays)
    trace = tmp_path/"trace.jsonl"; trace.write_text("synthetic")
    output = tmp_path/"result"
    monkeypatch.setattr("sys.argv", ["tool", "fakebag", "--mapper-trace", str(trace), "--output", str(output)])
    tool.main()
    summary = json.loads((output/"summary.json").read_text())
    assert summary["adopted_frames"] == 2
    assert summary["path"]["before_passage"]["completely_unknown"] == 1
    assert summary["path"]["before_passage"]["known_footprints"] == 1
    assert (output/"retained_features.npz").exists()


def test_low_span_clears_only_supported_observation():
    """複数画素の低高さ幅だけ解除し、1画素・未観測・閾値以上は保持する。"""
    arrays = dict(origin=np.array([0, 0]), resolution=np.array(1.0))
    for name in ("hazard",)+tool.CUES+("support_count","step_support_forward","step_support_rear"):
        arrays[name] = np.full((1, 5), np.nan)
    arrays["pixel_count"] = np.array([[2, 1, 0, 2, 2]])
    arrays["cell_min"] = np.array([[0., 0., np.nan, 0., 0.]])
    arrays["cell_max"] = np.array([[.02, 0., np.nan, .03, .1]])
    arrays["obstacle_height"][0, 4] = .1
    cells = [(i, 0) for i in range(6)]
    values, support = tool.extract(arrays, cells, obstacle_clear_height=.03)
    assert values[0, 3] == 0
    assert np.isnan(values[1:4, 3]).all()
    assert values[4, 3] == .1
    assert np.isnan(values[5]).all()
    legacy, _ = tool.extract(arrays, cells)
    assert np.isnan(legacy[0, 3])
    store = SpatialFeatureStore()
    store.update(cells, np.tile([np.nan, np.nan, np.nan, .1], (6, 1)), 10, np.ones((6, 4)))
    store.update(cells, values, 20, support)
    observed = store.query(cells, 20, None)
    assert observed[0, 3] == 0
    assert np.all(observed[1:, 3] == .1)
    assert store.data[cells[0]][1][3] == 20
    assert store.data[cells[1]][1][3] == 10


def test_cause_summary_keeps_overlap_and_matches_black_rounding():
    """複数cueの関与とNaNを保持し、100への丸めを主判定と一致させる。"""
    limits = np.array([20., .03, .07, .08])
    first = tool.describe(np.array([[20., 0., np.nan, .1], [0., .03, 0., 0.]]), limits)
    second = tool.describe(np.array([[0., 0., .07, 0.]]), limits)
    clear = tool.describe(np.full((1, 4), np.nan), limits)
    summary = tool.cause_summary([first, second, clear])
    assert summary["black_footprints"] == 2
    assert summary["overlapping_cue_footprints"] == dict.fromkeys(tool.CUES, 1)
    assert summary["exclusive_combinations"] == {
        "slope_deg+roughness+obstacle_height": 1, "step_height": 1}
    rounded = tool.describe(np.array([[19.91, 0., 0., 0.]]), limits)
    assert rounded["black_cells"] == rounded["slope_deg_black_cells"] == 1
