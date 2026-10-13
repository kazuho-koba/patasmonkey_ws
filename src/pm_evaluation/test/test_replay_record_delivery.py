"""再記録照合をROS起動なしで検査し、元データの重複や期間境界を保護する。"""
import importlib.util
from pathlib import Path
import sqlite3
import sys

from mcap.writer import CompressionType, Writer
import pytest


spec = importlib.util.spec_from_file_location(
    'delivery_comparison', Path(__file__).parents[1] / 'tools/replay_record_delivery.py')
comparison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(comparison)


def test_expected_boundary_and_duplicates(tmp_path):
    with (tmp_path / 'bag_0.mcap').open('wb') as stream:
        writer = Writer(stream, compression=CompressionType.NONE)
        writer.start(profile='ros2')
        schema = writer.register_schema(name='test', encoding='ros2msg', data=b'')
        channel = writer.register_channel(topic='/wit/imu', message_encoding='cdr', schema_id=schema)
        for stamp, payload in [(10, b'one'), (20, b'one'), (30, b'two'), (31, b'out')]:
            writer.add_message(channel_id=channel, log_time=stamp, publish_time=stamp, data=payload)
        writer.finish()
    expected = comparison.expected_messages(tmp_path, ['/wit/imu'], 20e-9)
    assert expected[comparison.signature('/wit/imu', b'one')] == 2
    assert expected[comparison.signature('/wit/imu', b'two')] == 1
    assert sum(expected.values()) == 3
    actual = comparison.recorded_messages(tmp_path, ['/wit/imu'], 'mcap')
    assert sum((actual - expected).values()) == 1
    missing = expected.copy()
    missing.subtract(actual)
    missing = +missing
    assert comparison.missing_examples(tmp_path, missing) == []
    candidate = comparison.Counter({comparison.signature('/wit/imu', b'two'): 1})
    example = comparison.missing_examples(tmp_path, candidate)[0]
    assert example['source_log_time_ns'] == 30


def test_sqlite_payload_and_missing_duplicate(tmp_path):
    with sqlite3.connect(str(tmp_path / 'bag_0.db3')) as connection:
        connection.execute('CREATE TABLE topics (id INTEGER, name TEXT)')
        connection.execute('CREATE TABLE messages (id INTEGER, topic_id INTEGER, data BLOB)')
        connection.execute('INSERT INTO topics VALUES (1, ?)', ('/wit/imu',))
        connection.execute('INSERT INTO messages VALUES (1, 1, ?)', (b'payload',))
    actual = comparison.recorded_messages(tmp_path, ['/wit/imu'], 'sqlite3')
    key = comparison.signature('/wit/imu', b'payload')
    expected = actual.copy()
    expected[key] += 1
    assert sum((expected - actual).values()) == 1
    assert comparison.signature('/oak/imu/data', b'payload') != key


def test_empty_selected_input_rejected(tmp_path):
    with pytest.raises(ValueError, match='選択topic'):
        comparison.expected_messages(tmp_path, ['/wit/imu'], 1)


@pytest.mark.parametrize('option,value', [('--duration', 'nan'), ('--duration', '0'),
                                         ('--domain', '0'), ('--qos-file', '/nonexistent/qos.yaml')])
def test_invalid_options_start_no_process(monkeypatch, tmp_path, option, value):
    monkeypatch.setattr(sys, 'argv', ['comparison', '--bag', str(tmp_path),
                                     '--output', str(tmp_path / 'result'), option, value])
    monkeypatch.setattr(comparison.signal, 'signal', lambda *args: None)
    def forbidden(*args, **kwargs):
        raise AssertionError('入力検査前にprocessを起動してはいけません')
    monkeypatch.setattr(comparison.subprocess, 'Popen', forbidden)
    with pytest.raises(SystemExit):
        comparison.main()
    assert not (tmp_path / 'result').exists()


def test_stop_handles_already_exited_process_group(monkeypatch):
    class Child:
        pid = 123456
        waited = False
        def poll(self):
            return None
        def wait(self):
            self.waited = True
    def already_exited(pid, signum):
        raise ProcessLookupError
    monkeypatch.setattr(comparison.os, 'killpg', already_exited)
    child = Child()
    comparison.stop(child)
    assert child.waited


@pytest.mark.parametrize('separate', [False, True])
def test_recording_groups_cover_input_once(separate):
    topics = comparison.IMU_TOPICS + comparison.IMAGE_TOPICS
    groups = comparison.recording_groups(topics, separate)
    flattened = [topic for _, members in groups for topic in members]
    assert flattened == topics
    assert len(flattened) == len(set(flattened))
    assert len(groups) == (2 if separate else 1)
    if separate:
        assert groups[0] == ('bag_imu', comparison.IMU_TOPICS)
        assert groups[1] == ('bag_images', comparison.IMAGE_TOPICS)


def test_cleanup_continues_after_one_stop_error(monkeypatch):
    calls = []
    def fake_stop(child):
        calls.append(child)
        if child == 'first':
            raise KeyboardInterrupt
    monkeypatch.setattr(comparison, 'stop', fake_stop)
    with pytest.raises(KeyboardInterrupt):
        comparison.stop_all(['first', 'second'])
    assert calls == ['first', 'second']
