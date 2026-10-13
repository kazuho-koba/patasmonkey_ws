"""到達確認済み件数と未観測指標を混同しない比較のテスト。"""
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    'compare_delivery_audits', Path(__file__).parents[1]/'tools/compare_delivery_audits.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_observed_missing_percentage():
    rows = module.comparison_rows([('a', {
        'topics': {'/imu': {'count': 98, 'header_active_hz': 99.8}},
        'independent_imu_probe': {'/imu': {'probe_count': 100, 'probe_only': 2}}})])
    assert rows[0]['observed_missing_percent'] == 2
    assert rows[0]['max_record_gap_s'] is None


def test_no_probe_is_not_zero_loss():
    rows = module.comparison_rows([('a', {'topics': {'/imu': {'count': 98}}})])
    assert rows[0]['observed_missing_percent'] is None
