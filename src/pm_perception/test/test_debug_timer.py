"""Hazard snapshot timerの周期制御を確認する。"""

from collections import deque
from types import SimpleNamespace

from pm_perception.depth_elevation_mapper_node import DepthElevationMapper


class _FakeClock:
    """timer callbackからstampを取得するための最小clock stub。"""

    def now(self):
        return self

    def to_msg(self):
        return "fixed-timer-stamp"


def _timer_state(has_fused_depth):
    """Node全体を初期化せず、timer callbackが使う状態だけを用意する。"""
    published_stamps = []
    state = SimpleNamespace(
        has_fused_depth=has_fused_depth,
        # 旧実装のdirty gateが誤って復活しても、このテストが検出する。
        grid_dirty=False,
        get_clock=lambda: _FakeClock(),
        publish_debug=published_stamps.append,
        debug_times_ms=deque(maxlen=10),
        window_debug_cycles=0,
    )
    return state, published_stamps


def test_timer_does_not_publish_before_first_fusion():
    """TF付きdepthを一度も融合していない空mapはpublishしない。"""
    state, published_stamps = _timer_state(has_fused_depth=False)

    DepthElevationMapper.debug_timer_callback(state)

    assert published_stamps == []
    assert state.window_debug_cycles == 0


def test_timer_publishes_after_first_fusion_even_without_new_data():
    """fusion後は新しい画像がないtimer tickもage更新のためpublishする。"""
    state, published_stamps = _timer_state(has_fused_depth=True)

    DepthElevationMapper.debug_timer_callback(state)

    assert published_stamps == ["fixed-timer-stamp"]
    assert state.window_debug_cycles == 1
