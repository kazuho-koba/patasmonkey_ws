"""単一画像gridの時間分離・one-shot・unknown保持を検証する。"""
import numpy as np
from pm_perception.single_frame_grid import SingleFrameGrid


def feed(grid, z, stamp):
    # 同じXYに2点。1枚内の高さ幅は10 cm、frame間のoffsetは別に変更する。
    grid.recenter(0, 0)
    grid.fuse_points(np.array([0.05, 0.05]), np.array([0.05, 0.05]),
                     np.array(z), stamp, 0.08, 0.03)
    return grid.stage2_layers(0.001, stamp, 0.15, 8.0)


def test_no_temporal_height_or_obstacle_fusion():
    grid = SingleFrameGrid(1, 1, 0.1)
    first = feed(grid, [0, 0.1], 1_000_000_000)
    second = feed(grid, [0.28, 0.38], 2_000_000_000)
    assert np.isclose(np.nanmax(first["obstacle_height"]), 0.1)
    assert np.isclose(np.nanmax(second["obstacle_height"]), 0.1)
    assert np.isclose(np.nanmin(second["elevation"]), 0.28)
    assert np.max(second["count"]) == 1


def test_confidence_wait_can_be_enabled():
    grid = SingleFrameGrid(1, 1, 0.1, single_frame_obstacle=False)
    assert not np.isfinite(feed(grid, [0, 0.1], 1_000_000_000)["obstacle_height"]).any()


def test_small_extent_and_empty_frame_remain_unknown():
    grid = SingleFrameGrid(1, 1, 0.1)
    layers = feed(grid, [0, 0.01], 1_000_000_000)
    assert not np.isfinite(layers["obstacle_height"]).any()
    grid.fuse_points(np.array([]), np.array([]), np.array([]), 2_000_000_000, 0.08, 0.03)
    assert not grid.stage2_layers(0.001, 2_000_000_000, 0.15, 8.0)["count"].any()
