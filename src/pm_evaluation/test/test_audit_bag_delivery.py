"""画像を展開しないheader抽出と、記録停滞・測定空白の区別を検証する。"""
from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from audit_bag_delivery import header_stamp, topic_summary, compare_diagnostic_headers, compare_probe_headers


def test_header_matches_foxy_serialization():
    """実際のROS serializerのstampとCDR先頭抽出が一致することを確認する。"""
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image
    image = Image()
    image.header.stamp.sec = 123
    image.header.stamp.nanosec = 456
    assert header_stamp(serialize_message(image), 'sensor_msgs/msg/Image') == 123000000456


def test_header_rejects_unknown_types_encodings_and_nanosec():
    """TF配列やCDR2を先頭Headerとして誤読しない。"""
    data = bytes([0, 1, 0, 0])+struct.pack('<iI', 10, 123)
    assert header_stamp(data, 'tf2_msgs/msg/TFMessage') is None
    assert header_stamp(bytes([0, 7, 0, 0])+data[4:], 'sensor_msgs/msg/Imu') is None
    assert header_stamp(data[:8], 'sensor_msgs/msg/Imu') is None
    bad = bytes([0, 1, 0, 0])+struct.pack('<iI', 10, 1000000000)
    assert header_stamp(bad, 'sensor_msgs/msg/Imu') is None


def test_big_endian_header():
    data = bytes([0, 0, 0, 0])+struct.pack('>iI', 10, 123)
    assert header_stamp(data, 'sensor_msgs/msg/Imu') == 10000000123


def test_record_stall_does_not_become_header_loss():
    rows = [(0, 0), (10000000, 500000000), (20000000, 500001000)]
    result = topic_summary(rows)
    assert result['max_record_gap_s'] == 0.5
    assert result['max_header_gap_s'] == 0.01


def test_headerless_event_topic():
    result = topic_summary([(None, 100), (None, 200)])
    assert result['header_count'] == 0
    assert 'header_active_hz' not in result


def test_source_diagnostic_missing_counts_keep_both_directions():
    rows = {'/oak/color/image_raw': [(1, 1), (2, 2), (4, 4)],
            '/oak/diagnostics/color_frame': [(1, 1), (3, 3), (4, 4)]}
    result = compare_diagnostic_headers(rows)['color']
    assert result['source_only'] == result['diagnostic_only'] == 1
    assert compare_diagnostic_headers(rows)['left']['status'] == 'missing_topic'


def test_partial_header_support_preserves_message_count():
    result = topic_summary([(None, 1), (100, 2), (200, 3)])
    assert result['count'] == 3 and result['header_count'] == 2


def test_independent_probe_comparison_clips_start_end(tmp_path):
    probe = tmp_path/'probe.csv'
    probe.write_text('topic,header_s\n/wit/imu,1\n/wit/imu,2\n/wit/imu,3\n/wit/imu,4\n')
    rows = {'/wit/imu': [(1000000000, 1), (3000000000, 3)]}
    result = compare_probe_headers(rows, probe)
    assert result['/wit/imu']['probe_only'] == 1
    assert result['/wit/imu']['bag_only'] == 0
    assert result['/oak/imu/data']['status'] == 'missing_topic'


def test_largest_gaps_preserve_position_and_units():
    from audit_bag_delivery import largest_record_gaps
    gaps = largest_record_gaps([(None, 10_000_000_000), (None, 12_000_000_000),
                                (None, 12_100_000_000)], 10_000_000_000, limit=1)
    assert gaps == [{'start_s': 0.0, 'end_s': 2.0, 'gap_s': 2.0}]
    assert largest_record_gaps([], 0) == []


def test_empty_steady_interval_has_no_invented_frequency():
    result = topic_summary([])
    assert result['count'] == 0
    assert result['record_hz'] is None and result['max_record_gap_s'] is None


def test_observed_retention_keeps_reference_limits_and_missing_bursts():
    from audit_bag_delivery import header_retention
    source = [(1, 1), (4, 4), (5, 5)]
    reference = [(stamp, stamp) for stamp in range(1, 6)]
    result = header_retention(source, reference)
    assert result['observed_retention_percent'] == 60.0
    assert result['reference_only'] == 2
    assert result['max_consecutive_reference_missing'] == 2
    assert result['max_missing_header_span_s'] == 1e-9
    assert result['all_publish_retention_percent'] is None
    assert header_retention([], reference)['status'] == 'missing_topic'


def test_observed_retention_counts_duplicates_and_common_bounds():
    from audit_bag_delivery import header_retention
    result = header_retention([(1, 1), (2, 2), (2, 3), (4, 4)],
                              [(0, 0), (1, 1), (2, 2), (4, 4), (5, 5)])
    assert result['matched_count'] == 3
    assert result['source_only'] == 1
    assert result['observed_retention_percent'] == 100.0
    assert result['common_header_start_ns'] == 1
    assert result['common_header_end_ns'] == 4


def test_record_gap_evidence_does_not_claim_ssd_or_publish_loss():
    from audit_bag_delivery import gap_evidence
    result = gap_evidence([(10_000_000, 1_000_000_000),
                           (20_000_000, 3_000_000_000)], 1_000_000_000)
    assert result[0]['record_gap_s'] == 2.0
    assert result[0]['header_gap_s'] == 0.01
    assert result[0]['receipt_age_change_s'] == 1.99


def test_retention_empty_reference_inside_overlapping_bounds():
    from audit_bag_delivery import header_retention
    result = header_retention([(1, 1), (3, 3)], [(0, 0), (4, 4)])
    assert result['reference_count'] == 0
    assert result['observed_retention_percent'] is None


def test_camera_info_comparison_uses_own_image_clock():
    from audit_bag_delivery import camera_info_header_pairs
    rows = {'/oak/color/image_raw': [(1, 1), (3, 3)],
            '/oak/color/camera_info': [(1, 1), (3, 3)],
            '/oak/depth/image_raw': [(2, 2), (4, 4)],
            '/oak/depth/camera_info': [(2, 2), (4, 4)]}
    result = camera_info_header_pairs(rows)
    assert result['color']['info_against_image']['observed_retention_percent'] == 100
    assert result['depth']['image_against_info']['observed_retention_percent'] == 100


def test_cli_combines_disjoint_bags_and_rejects_duplicate_topics(tmp_path):
    """実MCAPを使い、分離bagの合算・共通定常窓・二重計数拒否を確認する。"""
    import json
    import subprocess
    import yaml
    from mcap.writer import Writer, CompressionType
    script = Path(__file__).parents[1]/'tools/audit_bag_delivery.py'

    def fixture(name, topic, stamps):
        bag = tmp_path/name
        bag.mkdir()
        with (bag/'bag_0.mcap').open('wb') as stream:
            writer = Writer(stream, compression=CompressionType.NONE)
            writer.start(profile='ros2')
            schema = writer.register_schema(name='test/msg/Test', encoding='ros2msg', data=b'')
            channel = writer.register_channel(topic=topic, message_encoding='cdr', schema_id=schema)
            for stamp in stamps:
                writer.add_message(channel_id=channel, log_time=stamp*1_000_000_000,
                                   publish_time=stamp*1_000_000_000, data=b'fixture')
            writer.finish()
        (bag/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': {
            'storage_identifier': 'mcap', 'relative_file_paths': ['bag_0.mcap']}}))
        return bag

    main_bag = fixture('main', '/a', [0, 20, 40])
    bulk_bag = fixture('bulk', '/b', [15, 25])
    output = tmp_path/'result'
    result = subprocess.run([sys.executable, str(script), '--bag', str(main_bag),
                             '--additional-bag', str(bulk_bag), '--output', str(output),
                             '--end-s', '30'],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    summary = json.loads((output/'summary.json').read_text())
    assert summary['topics']['/a']['count'] == 3
    assert summary['steady_topics']['/a']['count'] == 1
    assert summary['steady_topics']['/b']['count'] == 2
    assert abs(summary['steady_interval']['end_s']-30) < 1e-12
    duplicate = fixture('duplicate', '/a', [25])
    result = subprocess.run([sys.executable, str(script), '--bag', str(main_bag),
                             '--additional-bag', str(duplicate), '--output', str(tmp_path/'bad')],
                            capture_output=True, text=True)
    assert result.returncode != 0 and '同じtopic' in result.stderr
    assert not (tmp_path/'bad').exists()
