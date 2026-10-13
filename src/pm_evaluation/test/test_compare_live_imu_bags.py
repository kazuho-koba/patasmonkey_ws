"""同時計測bagの境界・重複・値変更・probe到達をROSノードなしで検査する。"""
import importlib.util
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location(
    'live_imu_comparison', Path(__file__).parents[1] / 'tools/compare_live_imu_bags.py')
comparison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(comparison)


def test_common_interval_ignores_different_start_and_stop():
    left = [(0, 0, 'first'), (10, 10, 'a'), (20, 20, 'b'), (30, 30, 'last')]
    right = [(10, 11, 'a'), (20, 21, 'b')]
    result = comparison.compare_topic(left, right)
    assert result['only_mission'] == result['only_sidecar'] == 0
    assert result['mission']['count'] == result['sidecar']['count'] == 2


def test_duplicates_and_changed_payload_are_not_hidden():
    left = [(10, 10, 'a'), (20, 20, 'changed')]
    right = [(10, 11, 'a'), (10, 12, 'a'), (20, 21, 'original')]
    result = comparison.compare_topic(left, right)
    assert result['only_mission'] == 1
    assert result['only_sidecar'] == 2
    assert result['only_sidecar_header_ns'] == [10, 20]


def test_requested_stable_window_filters_both_sides():
    left = [(10, 10, 'start'), (20, 20, 'same'), (30, 30, 'end')]
    right = [(10, 11, 'different'), (20, 21, 'same'), (30, 31, 'end')]
    result = comparison.compare_topic(left, right, start_ns=15, end_ns=25)
    assert result['mission']['count'] == 1
    assert result['only_mission'] == result['only_sidecar'] == 0


def test_no_overlap_and_empty_topic_are_explicit():
    assert comparison.compare_topic([], [(10, 10, 'a')])['status'] == 'missing_topic'
    assert comparison.compare_topic([(10, 10, 'a')], [(20, 20, 'b')])['status'] == 'no_common_interval'


def test_probe_counts_only_received_missing_headers(tmp_path):
    csv_path = tmp_path / 'probe.csv'
    csv_path.write_text('topic,header_s\n/wit/imu,1.00000002\n')
    results = {'/wit/imu': dict(status='ok', only_sidecar_header_ns=[1000000020, 1000000030])}
    comparison.add_probe_evidence(results, csv_path)
    assert results['/wit/imu']['only_sidecar_seen_probe'] == 1


def test_missing_metadata_is_rejected(tmp_path):
    with pytest.raises(ValueError, match='metadata'):
        comparison.read_imu(tmp_path)


def test_probe_exposes_missing_in_both_bags_without_calling_sidecar_truth(tmp_path):
    path = tmp_path / 'probe.csv'
    path.write_text('topic,header_s\n/wit/imu,1.0\n/wit/imu,1.00000001\n/wit/imu,1.00000002\n')
    left = {'/wit/imu': [(1000000000, 0, 'a'), (1000000020, 0, 'c')]}
    right = {'/wit/imu': [(1000000000, 1, 'a'), (1000000020, 1, 'c')]}
    result = {'/wit/imu': comparison.compare_topic(left['/wit/imu'], right['/wit/imu'])}
    comparison.add_probe_evidence(result, path, left, right)
    assert result['/wit/imu']['only_sidecar'] == 0
    assert result['/wit/imu']['probe_header_count'] == 3
    assert result['/wit/imu']['probe_headers_not_mission'] == 1
    assert result['/wit/imu']['probe_headers_not_sidecar'] == 1


def test_record_order_clock_regression_not_hidden_by_header_sort():
    values = [(10, 200, 'later_receipt'), (20, 100, 'earlier_receipt')]
    result = comparison.metrics(values)
    assert result['record_order_nonpositive_header_intervals'] == 1
    assert result['max_log_gap_s'] == pytest.approx(100e-9)
