"""独立frameの初期化・時間融合対照・pose補間をsynthetic入力で検証する。"""
import numpy as np

from evaluate_single_depth_terrain import evaluate_points
from evaluate_independent_depth_frames import interpolate_pose
from evaluate_independent_depth_frames import recorded_transform
from evaluate_independent_depth_frames import evaluate_footprint
from geometry_msgs.msg import Transform
from pm_perception.mapper_replay_trace import transform_record
from pm_perception.depth_projection import transform_points
from pm_perception.rolling_elevation_grid import RollingElevationGrid


def parameters():
    return dict(map_size_x=4.0, map_size_y=4.0, resolution=.1,
                ground_merge_threshold=.2, obstacle_min_height=.03,
                measurement_variance=.0025, obstacle_confidence_min=.15,
                observation_decay_time=8.0, feature_max_observation_age=3.0,
                feature_neighborhood_radius_cells=1, feature_min_neighbors=5,
                hazard_slope_limit_deg=20.0, hazard_roughness_limit=.03,
                hazard_step_limit=.07, hazard_obstacle_height_limit=.08,
                step_min_side_neighbors=2)


def plane(z):
    x, y = np.meshgrid(np.arange(-10, 10)*.1+.05, np.arange(-10, 10)*.1+.05)
    return np.column_stack((x.ravel(), y.ravel(), np.full(x.size, z)))


def test_independent_flat_frames_do_not_inherit_height_shift():
    """フレーム間の高さ差は独立評価には入らず、融合対照には障害物証拠として残る。"""
    p = parameters()
    first = evaluate_points(plane(-.12), p, 1_000_000_000)
    second = evaluate_points(plane(-.40), p, 1_100_000_000)
    assert np.nanmax(first["hazard"]) < .001
    assert np.nanmax(second["hazard"]) < .001
    grid = RollingElevationGrid(4, 4, .1)
    evaluate_points(plane(-.12), p, 1_000_000_000, grid=grid)
    fused = evaluate_points(plane(-.40), p, 1_100_000_000, grid=grid)
    assert np.nanmax(fused["hazard"]) == 1.0


def test_obstacle_gate_and_raw_reference_are_distinct():
    """1枚の高さ幅はあっても現行confidence .1は最低.15を満たさない。"""
    p = parameters()
    points = plane(-.12)
    points = np.vstack([points, points+np.array([0, 0, .10])])
    arrays = evaluate_points(points, p, 1_000_000_000)
    assert not np.any(np.isfinite(arrays["obstacle_height"]))
    assert np.nanmax(arrays["hazard"]) < .001
    assert np.nanmax(arrays["reference_hazard"]) == 1.0
    assert np.max(arrays["pixel_count"]) == 2


def test_footprint_diagnostics_preserve_missing_and_positions():
    """診断追加で旧統計を変えず、欠測はJSON nullとして保持する。"""
    import json
    p = parameters()
    arrays = evaluate_points(plane(-.12), p, 1_000_000_000)
    pose = (1_000_000_000, 0., 0., 0., 0., 0., 0.)
    original = evaluate_footprint(arrays, pose, .45, .55, p)
    detailed = evaluate_footprint(arrays, pose, .45, .55, p, cell_diagnostics=True)
    data = json.loads(detailed.pop("cell_diagnostics_json"))
    assert detailed == original
    assert len(data["x"]) == original["hazard_cells"]
    assert all(v is None for v in data["obstacle_height"])
    assert all(v >= 5 for v in data["support_count"])


def test_pose_slerp_no_extrapolation_or_large_gap():
    times = np.array([1_000_000_000, 1_100_000_000], dtype=np.int64)
    poses = np.array([[0, 0, 0, 0, 0, 0, 1], [1, 0, 0, 0, 0, 0, -1]], dtype=float)
    result = interpolate_pose(times, poses, 1_050_000_000, .25)
    assert np.allclose(result[:3], [.5, 0, 0])
    assert np.isclose(abs(result[-1]), 1)
    assert interpolate_pose(times, poses, 999_000_000, .25) is None
    assert interpolate_pose(times, poses, 1_050_000_000, .05) is None


def test_recorded_camera_tf_is_used_without_pose_recomposition():
    """保存→読込したcamera TFによる点投影が元の使用TFと数値一致する。"""
    tf = Transform()
    tf.translation.x, tf.translation.z = 1.23456789, -.123456789
    tf.rotation.x, tf.rotation.w = .0123456789, .999
    points = np.array([[.2, .3, 2.0], [-.4, .1, 3.0]], dtype=np.float32)
    original = transform_points(points, tf)
    reused = transform_points(points, recorded_transform(transform_record(tf)))
    assert np.array_equal(original, reused)
