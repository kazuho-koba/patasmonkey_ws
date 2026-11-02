"""原本非変更・時刻異常・一括解析とroot制限の回帰試験。"""
import json
from pathlib import Path
import struct
import time

from mcap.writer import Writer, CompressionType
import pytest
import yaml

from pm_evaluation.bag_inspection import inspect
from pm_inspector.jobs import Jobs


def fixture(root, name='rosbag2_test', expected=3):
    bag = root/name
    bag.mkdir()
    with (bag/'data_0.mcap').open('wb') as stream:
        writer = Writer(stream, compression=CompressionType.NONE)
        writer.start(profile='ros2')
        schema = writer.register_schema(name='sensor_msgs/msg/Imu', encoding='ros2msg', data=b'')
        channel = writer.register_channel(topic='/imu', message_encoding='cdr', schema_id=schema)
        for i, header in enumerate((100, 100, 99)):
            writer.add_message(channel_id=channel, log_time=(100+i)*10**9,
                               publish_time=0, data=b'\x00\x01\x00\x00'+struct.pack('<iI', header, 0))
        writer.finish()
    (bag/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': dict(
        storage_identifier='mcap', relative_file_paths=['data_0.mcap'], message_count=expected,
        starting_time={'nanoseconds_since_epoch': 100*10**9}, duration={'nanoseconds': 2*10**9},
        topics_with_message_count=[dict(topic_metadata=dict(name='/imu', type='sensor_msgs/msg/Imu'),
                                       message_count=expected)])}))
    return bag


def test_single_pass_statistics_and_original_unchanged(tmp_path):
    bag = fixture(tmp_path)
    original = {p.name: p.read_bytes() for p in bag.iterdir()}
    report = inspect(bag, tmp_path/'result', window_sec=1.)
    row = report['topics']['/imu']
    assert report['integrity_verified']
    assert row['average_hz'] == 1.
    assert row['header_duplicates'] == row['header_backwards'] == 1
    assert row['window_hz'] == [1., 2.]
    assert sum(row['window_hz']) == row['count']
    assert (tmp_path/'result/completed.json').is_file()
    assert original == {p.name: p.read_bytes() for p in bag.iterdir()}


def test_metadata_mismatch_is_reported(tmp_path):
    report = inspect(fixture(tmp_path, expected=4), tmp_path/'result')
    assert not report['integrity_verified']
    assert report['differences']['/imu'] == dict(expected=4, actual=3)


def test_sqlite_read_only_inspection(tmp_path):
    import sqlite3
    bag = fixture(tmp_path)
    with sqlite3.connect(str(bag/'data_0.db3')) as db:
        db.executescript('CREATE TABLE topics (id INTEGER,name TEXT,type TEXT); '
                         'CREATE TABLE messages (id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB); '
                         "INSERT INTO topics VALUES(1,'/imu','sensor_msgs/msg/Imu');")
        for i in range(3):
            db.execute('INSERT INTO messages VALUES(?,?,?,?)',
                       (i+1, 1, (100+i)*10**9, b'\x00\x01\x00\x00'+struct.pack('<iI',100+i,0)))
    path = bag/'metadata.yaml'
    info = yaml.safe_load(path.read_text())
    info['rosbag2_bagfile_information'].update(storage_identifier='sqlite3', relative_file_paths=['data_0.db3'])
    path.write_text(yaml.safe_dump(info))
    report = inspect(bag, tmp_path/'sqlite_result')
    assert report['integrity_verified']
    assert report['topics']['/imu']['average_hz'] == 1.


def test_mcap_crc_corruption_has_no_completion_marker(tmp_path):
    bag = fixture(tmp_path)
    path = bag/'data_0.mcap'
    data = bytearray(path.read_bytes())
    position = data.find(b'\x00\x01\x00\x00'+struct.pack('<iI', 100, 0))
    assert position >= 0
    data[position+4] ^= 1
    path.write_bytes(data)
    with pytest.raises(Exception):
        inspect(bag, tmp_path/'corrupt_result')
    assert not (tmp_path/'corrupt_result/completed.json').exists()


def test_unsafe_paths_and_existing_result_are_rejected(tmp_path):
    bag = fixture(tmp_path)
    with pytest.raises(ValueError):
        inspect(bag, bag/'result')
    out = tmp_path/'result'
    out.mkdir()
    with pytest.raises(FileExistsError):
        inspect(bag, out)
    jobs = Jobs(bag, tmp_path/'results')
    with pytest.raises(ValueError):
        jobs.resolve(str(tmp_path))


def test_batch_subprocess_and_results(tmp_path):
    bags = [fixture(tmp_path, 'rosbag2_a'), fixture(tmp_path, 'rosbag2_b')]
    jobs = Jobs(tmp_path, tmp_path/'results')
    listed = jobs.dispatch('list', {})
    assert len(listed['bags']) == 2
    jobs.dispatch('start', dict(bags=[str(b) for b in bags], window_sec=1))
    try:
        jobs.thread.join(timeout=20.)
        assert not jobs.thread.is_alive()
        status = jobs.dispatch('status', {})
        assert status['state'] == 'DONE', status
        for i in range(2):
            report = jobs.dispatch('result', dict(index=i))
            assert report['integrity_verified']
            assert 'window_hz' not in report['topics']['/imu']
            topic = jobs.dispatch('result', dict(index=i, topic='/imu'))
            assert topic['statistics']['window_hz'] == [1., 2.]
    finally:
        jobs.close()
    history = jobs.dispatch('history', {})['jobs']
    assert len(history) == 1
    reopened = Jobs(tmp_path, tmp_path/'results')
    reopened.dispatch('load', dict(job=history[0]))
    assert reopened.dispatch('result', dict(index=1))['integrity_verified']


def test_cancel_only_owned_worker(tmp_path, monkeypatch):
    """今回所有する子だけに停止signalを送り、未解析bagを完了扱いしない。"""
    import threading
    import signal
    import pm_inspector.jobs as module
    bag = fixture(tmp_path)
    started, stopped = threading.Event(), threading.Event()
    signals = []

    class Child:
        def __init__(self, *args, **kwargs):
            started.set()

        def poll(self):
            return -signal.SIGTERM if stopped.is_set() else None

        def send_signal(self, value):
            signals.append(value)
            stopped.set()

        def wait(self):
            assert stopped.wait(5.)
            return -signal.SIGTERM

    monkeypatch.setattr(module.subprocess, 'Popen', Child)
    jobs = Jobs(tmp_path, tmp_path/'results')
    jobs.dispatch('start', dict(bags=[str(bag)]))
    assert started.wait(5.)
    jobs.dispatch('cancel', {})
    jobs.thread.join(5.)
    assert jobs.state['state'] == 'CANCELLED'
    assert signals == [signal.SIGTERM]
    assert not (Path(jobs.state['entries'][0]['output'])/'completed.json').exists()
