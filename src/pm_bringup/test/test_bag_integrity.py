"""metadataだけ成功する未保存bagを、センサを起動せず実ファイルで検出する。"""
import sqlite3
import yaml
from mcap.writer import Writer, CompressionType
from pm_bringup.bag_integrity import verify_message_counts


def metadata(directory, storage, filename, count):
    (directory/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': {
        'storage_identifier': storage, 'relative_file_paths': [filename], 'message_count': count,
        'topics_with_message_count': [{'topic_metadata': {'name': '/test'}, 'message_count': count}]}}))


def mcap_fixture(path, count):
    with path.open('wb') as stream:
        writer = Writer(stream, compression=CompressionType.NONE)
        writer.start(profile='ros2')
        schema = writer.register_schema(name='test', encoding='ros2msg', data=b'')
        channel = writer.register_channel(topic='/test', message_encoding='cdr', schema_id=schema)
        for index in range(count):
            writer.add_message(channel_id=channel, log_time=index, publish_time=index, data=b'fixture')
        writer.finish()


def test_mcap_counts_and_metadata_only_failure(tmp_path):
    mcap_fixture(tmp_path/'bag_0.mcap', 2)
    metadata(tmp_path, 'mcap', 'bag_0.mcap', 2)
    assert verify_message_counts(tmp_path)['verified']
    metadata(tmp_path, 'mcap', 'bag_0.mcap', 3)
    result = verify_message_counts(tmp_path)
    assert not result['verified'] and result['actual_total'] == 2


def test_empty_mcap_cannot_satisfy_positive_metadata(tmp_path):
    mcap_fixture(tmp_path/'bag_0.mcap', 0)
    metadata(tmp_path, 'mcap', 'bag_0.mcap', 82679)
    result = verify_message_counts(tmp_path)
    assert not result['verified'] and result['actual_total'] == 0


def test_sqlite_read_only_counts(tmp_path):
    with sqlite3.connect(str(tmp_path/'bag_0.db3')) as database:
        database.executescript('CREATE TABLE topics (id INTEGER, name TEXT); '
                               'CREATE TABLE messages (topic_id INTEGER, data BLOB); '
                               "INSERT INTO topics VALUES (1, '/test'); INSERT INTO messages VALUES (1, X'00');")
    metadata(tmp_path, 'sqlite3', 'bag_0.db3', 1)
    assert verify_message_counts(tmp_path)['verified']


def test_missing_split_file_fails(tmp_path):
    metadata(tmp_path, 'mcap', 'missing.mcap', 1)
    result = verify_message_counts(tmp_path)
    assert not result['verified'] and result['error']


def test_outside_directory_and_unsupported_storage_fail(tmp_path):
    metadata(tmp_path, 'mcap', '../outside.mcap', 1)
    assert not verify_message_counts(tmp_path)['verified']
    (tmp_path/'fixture').write_text('not a bag')
    metadata(tmp_path, 'unknown', 'fixture', 1)
    assert not verify_message_counts(tmp_path)['verified']


def test_sidecar_info_success_does_not_override_content_failure(tmp_path, monkeypatch):
    """bag infoの成功を模擬しても実件数不一致なら専用bagのverifiedはfalse。"""
    from types import SimpleNamespace
    import pm_bringup.record_trial as module
    monkeypatch.setattr(module.subprocess, 'run', lambda *a, **k:
                        SimpleNamespace(returncode=0, stdout='fixture', stderr=''))
    mcap_fixture(tmp_path/'bag_0.mcap', 0)
    metadata(tmp_path, 'mcap', 'bag_0.mcap', 10)
    result = module.verify_recorder(tmp_path, SimpleNamespace(poll=lambda: 0, returncode=0))
    assert not result['verified'] and not result['content_integrity']['verified']
