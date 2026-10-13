"""再生範囲とMCAP topic選択を、ROSノードを起動せず確認する。"""
import sys
from mcap.writer import Writer, CompressionType
import pytest
from pm_evaluation.cli import bag_clock_player


def test_split_bag_uses_numeric_order(tmp_path):
    """10分割を超えても時刻を逆行させず、元のpayloadを順に配信する。"""
    for index in [10, 2, 0, 1]:
        with (tmp_path / ('bag_%d.mcap' % index)).open('wb') as stream:
            writer = Writer(stream, compression=CompressionType.NONE)
            writer.start(profile='ros2')
            schema = writer.register_schema(name='test', encoding='ros2msg', data=b'')
            channel = writer.register_channel(topic='/wit/imu', message_encoding='cdr', schema_id=schema)
            writer.add_message(channel_id=channel, log_time=index+1,
                               publish_time=index+1, data=str(index).encode())
            writer.finish()
    records = list(bag_clock_player.iter_records(tmp_path, ['/wit/imu']))
    assert [row[0] for row in records] == [1, 2, 3, 11]
    assert [row[2] for row in records] == [b'0', b'1', b'2', b'10']


def test_selected_records_keep_original_times_and_payload(tmp_path):
    """非選択messageを除いても、選択messageの順序・時刻・dataは不変。"""
    with (tmp_path/'bag_0.mcap').open('wb') as stream:
        writer = Writer(stream, compression=CompressionType.NONE)
        writer.start(profile='ros2')
        schema = writer.register_schema(name='test', encoding='ros2msg', data=b'')
        chosen = writer.register_channel(topic='/wit/imu', message_encoding='cdr', schema_id=schema)
        ignored = writer.register_channel(topic='/unused', message_encoding='cdr', schema_id=schema)
        writer.add_message(channel_id=chosen, log_time=10, publish_time=10, data=b'first')
        writer.add_message(channel_id=ignored, log_time=20, publish_time=20, data=b'ignore')
        writer.add_message(channel_id=chosen, log_time=30, publish_time=30, data=b'last')
        writer.finish()
    assert list(bag_clock_player.iter_records(tmp_path, ['/wit/imu'])) == [
        (10, '/wit/imu', b'first'), (30, '/wit/imu', b'last')]


def test_duration_is_optional_and_uses_bag_seconds(monkeypatch):
    monkeypatch.setattr(sys, 'argv', ['player', '/tmp/bag', '--topic', '/wit/imu'])
    assert bag_clock_player.parse_args().duration is None
    monkeypatch.setattr(sys, 'argv', ['player', '/tmp/bag', '--topic', '/wit/imu', '--duration', '2.5'])
    assert bag_clock_player.parse_args().duration == 2.5


@pytest.mark.parametrize('duration', ['0', '-1', 'nan', 'inf'])
def test_invalid_duration_fails_before_ros_init(monkeypatch, duration):
    monkeypatch.setattr(sys, 'argv', ['player', '/tmp/bag', '--topic', '/wit/imu', '--duration', duration])
    with pytest.raises(SystemExit):
        bag_clock_player.main()


@pytest.mark.parametrize('drain', ['-1', 'nan', 'inf'])
def test_invalid_drain_fails_before_ros_init(monkeypatch, drain):
    monkeypatch.setattr(sys, 'argv', ['player', '/tmp/bag', '--topic', '/wit/imu',
                                     '--drain-seconds', drain])
    with pytest.raises(SystemExit):
        bag_clock_player.main()
