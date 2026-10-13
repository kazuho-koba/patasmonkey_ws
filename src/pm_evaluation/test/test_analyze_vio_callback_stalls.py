"""独立購読を使わないVIO停滞の事後照合を、ROS不要で検査する。"""
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location('callback_stalls', Path(__file__).parents[1]/'tools/analyze_vio_callback_stalls.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def event():
    return dict(stamp=1.008, header_delta_ms=8.0, callback_gap_ms=60.0,
                previous_feed_ms=0.005, previous_odometry_ms=0.2)


def test_parse_ignores_other_warnings_and_incomplete_rows():
    text = ('[IMU_INPUT_GAP] delta=0.18\n'
            '[IMU_CALLBACK_DELAY] callback_gap_ms=60.0 header_delta_ms=8.0 previous_feed_ms=0.005 previous_odometry_ms=0.2 stamp=1.008\n'
            '[IMU_CALLBACK_DELAY] stamp=1.0\n')
    assert module.parse_delays(text) == [event()]


def test_same_headers_compared_not_assumed_same_receipt_clock():
    row = module.match_record_intervals([event()], {1_000_000_000:2_000_000_000, 1_008_000_000:2_008_000_000})[0]
    assert row['source_record_gap_ms'] == 8.0
    assert row['interval_difference_ms'] == 52.0


def test_missing_endpoint_not_interpolated_or_claimed_loss():
    assert module.match_record_intervals([event()], {1_008_000_000:2_008_000_000})[0]['source_record_gap_ms'] is None


def test_rounding_tolerance_and_empty_input():
    assert module.nearest_stamp([1_000_000_000], 1_000_001_999) == 1_000_000_000
    assert module.nearest_stamp([1_000_000_000], 1_000_002_001) is None
    assert module.nearest_stamp([], 1) is None


def test_record_clock_reordering_is_not_hidden():
    row = module.match_record_intervals([event()], {1_000_000_000:2_008_000_000, 1_008_000_000:2_000_000_000})[0]
    assert row['source_record_gap_ms'] == -8.0


def test_existing_driver_clock_is_separate_from_recorder_clock():
    row = module.match_record_intervals([event()], {}, {1_000_000_000:3_000_000_000, 1_008_000_000:3_060_000_000})[0]
    assert row['source_record_gap_ms'] is None
    assert row['driver_post_publish_gap_ms'] == 60.0


def test_record_stall_not_assumed_driver_or_sensor_stall():
    records = {1_000_000_000:2_000_000_000, 1_008_000_000:2_800_000_000}
    driver = {1_000_000_000:3_000_000_000, 1_008_000_000:3_008_000_000}
    row = module.largest_record_gap_evidence(records, driver)[0]
    assert row['record_gap_ms'] == 800.0
    assert row['header_gap_ms'] == 8.0
    assert row['driver_post_publish_gap_ms'] == 8.0
    assert module.largest_record_gap_evidence(records, {})[0]['driver_post_publish_gap_ms'] is None
    assert module.largest_record_gap_evidence({}, {}) == []


def test_missing_vio_samples_were_published_over_time_or_in_burst():
    gaps = module.parse_input_gaps('[IMU_INPUT_GAP] previous=1.0 current=1.024 delta=0.024 receipt=2.0')
    records = {1_000_000_000:2_000_000_000, 1_008_000_000:2_001_000_000,
               1_016_000_000:2_002_000_000, 1_024_000_000:2_003_000_000}
    driver = {1_000_000_000:3_000_000_000, 1_008_000_000:3_008_000_000,
              1_016_000_000:3_016_000_000, 1_024_000_000:3_024_000_000}
    row = module.match_input_gaps(gaps, records, driver)[0]
    assert row['source_record_internal_count'] == 2
    assert row['source_record_internal_span_ms'] == 1.0
    assert row['driver_post_publish_internal_count'] == 2
    assert row['driver_post_publish_internal_span_ms'] == 8.0
    assert row['driver_post_publish_endpoint_span_ms'] == 24.0
    assert module.match_input_gaps(gaps, {}, {})[0]['source_record_internal_count'] is None
