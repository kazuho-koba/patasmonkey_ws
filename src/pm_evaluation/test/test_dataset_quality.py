"""固定時間窓で初期欠測・途絶を隠さないことを確認する。"""
from copy import deepcopy
import struct

import pytest
import yaml
from mcap.writer import Writer
from pm_evaluation.cli.analyze_dataset_quality import (
    analyze, gap_in_window, intervals, load_policy, main, read_bag)


def policy():
    return {'version': 1, 'window_sec': 10.0, 'step_sec': 1.0,
            'consecutive_windows': 3,
            'topics': {'/imu': {'required': True, 'expected_hz': 100.,
                                'header': True, 'clock': 'record',
                                'max_gap_sec': .03, 'max_delay_sec': .1}}}


def samples(start=0, end=20, hz=100):
    return [(1_000_000_000 + round(i*1e9/hz),
             1_000_000_000 + round(i*1e9/hz))
            for i in range(round(start*hz), round(end*hz))]


def test_normal_and_confirmation():
    _, result = analyze(10**9, 20., {'/imu': samples()}, {'/imu': 0}, policy())
    assert result['stable_start_sec'] == 0
    assert result['confirmed_at_sec'] == 12


def test_initial_gap_is_not_removed_from_window():
    _, result = analyze(10**9, 20., {'/imu': samples(5)}, {'/imu': 0}, policy())
    assert result['stable_start_sec'] == 5
    assert not result['global_windows'][0]['passed']


def test_stopped_topic_does_not_pass_on_short_span():
    _, result = analyze(10**9, 20., {'/imu': samples(0, 3)}, {'/imu': 0}, policy())
    assert result['stable_start_sec'] is None
    assert 'no_messages' in result['global_windows'][-1]['reasons'][0]


def test_gap_crossing_boundary():
    assert gap_in_window([0., .95, 1.05, 2.], 1., 1.04) == pytest.approx(.1)


def test_missing_and_unconfigured():
    p = policy()
    p['topics']['/imu']['expected_hz'] = None
    _, result = analyze(10**9, 20., {'/imu': []}, {'/imu': 0}, p)
    assert result['status'] == 'no_qualified_interval'
    assert 'expected_hz_unconfigured' in result['global_windows'][0]['reasons'][0]


def test_header_reverse_and_delay():
    values = samples()
    values[500] = (values[500][0], values[499][1])
    _, result = analyze(10**9, 20., {'/imu': values}, {'/imu': 0}, policy())
    assert 'header_not_increasing' in result['global_windows'][0]['reasons'][0]
    values = [(r, h-200_000_000) for r, h in samples()]
    _, result = analyze(10**9, 20., {'/imu': values}, {'/imu': 0}, policy())
    assert result['stable_start_sec'] is None
    assert result['rate_only_stable_start_sec'] == 0


def test_stereo_missing_pairs():
    p = policy()
    p['topics']['/right'] = deepcopy(p['topics']['/imu'])
    p['stereo'] = {'left': '/imu', 'right': '/right', 'required': True,
                   'max_delta_sec': .001, 'min_match_ratio': .99}
    shifted = [(r, h+5_000_000) for r, h in samples()]
    _, result = analyze(10**9, 20., {'/imu': samples(), '/right': shifted},
                        {'/imu': 0, '/right': 0}, p)
    assert 'stereo_unmatched' in result['global_windows'][0]['reasons']


def test_short_bag_is_unassessed():
    _, result = analyze(10**9, 2., {'/imu': samples(0, 2)}, {'/imu': 0}, policy())
    assert result['global_windows'] == []
    assert result['stable_start_sec'] is None


def fixture_bag(tmp_path):
    bag = tmp_path/'bag'
    bag.mkdir()
    values = samples()
    with (bag/'data.mcap').open('wb') as stream:
        writer = Writer(stream)
        writer.start()
        schema = writer.register_schema(name='sensor_msgs/msg/Imu', encoding='ros2msg', data=b'')
        channel = writer.register_channel(topic='/imu', message_encoding='cdr', schema_id=schema)
        for r, h in values:
            data = b'\x00\x01\x00\x00' + struct.pack('<iI', h//10**9, h % 10**9)
            writer.add_message(channel_id=channel, log_time=r, publish_time=r, data=data)
        writer.finish()
    meta = {'rosbag2_bagfile_information': {
        'storage_identifier': 'mcap', 'relative_file_paths': ['data.mcap'],
        'starting_time': {'nanoseconds_since_epoch': 10**9},
        'duration': {'nanoseconds': 20*10**9},
        'topics_with_message_count': [{'topic_metadata': {'name': '/imu'}, 'message_count': len(values)}]}}
    (bag/'metadata.yaml').write_text(yaml.safe_dump(meta))
    return bag


def test_mcap_end_to_end_and_overwrite_protection(tmp_path):
    bag = fixture_bag(tmp_path)
    path = tmp_path/'policy.yaml'
    path.write_text(yaml.safe_dump(policy()))
    assert main([str(bag), '--policy', str(path)]) == 0
    result = yaml.safe_load((bag/'quality/summary.yaml').read_text())
    assert result['stable_start_sec'] == 0
    assert (bag/'quality/timeline.png').is_file()
    assert (bag/'quality/completed.json').is_file()
    with pytest.raises(SystemExit):
        main([str(bag), '--policy', str(path)])


def test_missing_split_is_error(tmp_path):
    bag = fixture_bag(tmp_path)
    (bag/'data.mcap').rename(bag/'other.mcap')
    with pytest.raises(ValueError, match='欠損'):
        read_bag(bag, policy())


def test_invalid_policy(tmp_path):
    p = policy()
    p['window_sec'] = 0
    path = tmp_path/'policy.yaml'
    path.write_text(yaml.safe_dump(p))
    with pytest.raises(ValueError):
        load_policy(path)


def test_string_boolean_is_rejected(tmp_path):
    p = policy()
    p['topics']['/imu']['required'] = 'false'
    path = tmp_path/'policy.yaml'
    path.write_text(yaml.safe_dump(p))
    with pytest.raises(ValueError):
        load_policy(path)
