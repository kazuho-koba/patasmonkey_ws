"""GUI区間境界の分割・未完了eventの扱いを確認する。"""
from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from compare_gui_stages import stages, requirements_failures, coverage_failures


def test_camera_off_on_boundaries():
    events = [dict(event=kind, mode='offon', wall_time=stamp) for kind, stamp in
              [('begin', 100), ('camera_off_end', 125), ('camera_on_begin', 125), ('end', 150)]]
    assert stages(events) == [('camera_off', 103, 123), ('camera_on', 128, 148)]


def test_incomplete_event_is_not_a_success():
    with pytest.raises(ValueError):
        stages([dict(event='begin', mode='baseline', wall_time=100)])


def test_missing_vio_topic_fails_minimum_rate():
    failures = requirements_failures([], [('full', 100, 120)], [('/ov_msckf/odomimu', 100)])
    assert failures[0]['recorded_hz'] == 0


def test_healthy_vio_meets_minimum_rate():
    assert requirements_failures([dict(stage='full', topic='/ov_msckf/odomimu', window_hz=126)],
                                 [('full', 100, 120)], [('/ov_msckf/odomimu', 100)]) == []


def test_pre_record_time_is_not_valid_performance_window():
    assert coverage_failures([('baseline', 100, 120)], 104, 130)[0]['stage'] == 'baseline'
    assert coverage_failures([('full', 105, 120)], 104, 130) == []
