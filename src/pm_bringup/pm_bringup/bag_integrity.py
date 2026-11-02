"""停止済みbagの軽量確認と、明示実行専用の本体走査を提供する。

通常停止はwriterのclose/flush集計とmetadataを照合し、本体は開かない。
CLIの全件検査だけはMCAP record走査やSQLite件数確認を行う。
どちらもセンサがpublishした総数の証明ではない。
"""
from collections import Counter
from contextlib import closing
from pathlib import Path
import sqlite3
import yaml


def verify_recording_summary(directory):
    """停止・close済みwriterの集計とmetadataを照合し、本体は開かない。

    verifiedは保存手順と集計の整合性を意味し、CRCや本体破損の検査ではない。
    周波数はwriter入口の受信区間平均で、未受信topicや1件だけの場合は未定義。
    """
    directory = Path(directory)
    result = {'verified': False, 'verification_mode': 'metadata_and_writer',
              'full_scan_performed': False, 'topics': {}, 'error': ''}
    try:
        info = yaml.safe_load((directory/'metadata.yaml').read_text())['rosbag2_bagfile_information']
        writer = yaml.safe_load((directory/'async_writer.yaml').read_text())
        if (writer.get('verified') is not True or writer.get('error')
                or writer.get('transport_error')
                or writer['accepted'] != writer['written']
                or writer['accepted_bytes'] != writer['written_bytes']):
            raise ValueError('writerの終了・flush証明が不正です')
        expected = {}
        for entry in info['topics_with_message_count']:
            name = entry['topic_metadata']['name']
            if name in expected or not entry['topic_metadata']['type']:
                raise ValueError('metadataのtopic定義が不正です')
            expected[name] = int(entry['message_count'])
        if any(n < 0 for n in expected.values()):
            raise ValueError('metadataの件数が負です')
        total = int(info['message_count'])
        if total != sum(expected.values()) or total != writer['written']:
            raise ValueError('metadataとwriterの総件数が一致しません')
        topics = writer['topics']
        for name in set(expected) | set(topics):
            sample = topics.get(name, {})
            count = sample.get('written', 0)
            if count != sample.get('accepted', 0) or count != expected.get(name, 0):
                raise ValueError('topic別件数が一致しません: '+name)
            result['topics'][name] = dict(sample, message_count=count)
        if sum(t['written'] for t in topics.values()) != total:
            raise ValueError('writerのtopic集計と総件数が一致しません')
        visited = set()
        for name in info['relative_file_paths']:
            path = (directory/name).resolve()
            path.relative_to(directory.resolve())
            if path in visited or not path.is_file() or path.stat().st_size <= 0:
                raise ValueError('保存ファイルの不足・重複・空ファイル: '+name)
            visited.add(path)
        if not visited:
            raise ValueError('保存ファイルがありません')
        result.update(verified=True, expected_total=total, writer_total=writer['written'])
    except (OSError, ValueError, TypeError, KeyError, AttributeError, yaml.YAMLError) as error:
        result['error'] = str(error)
    return result


def verify_message_counts(directory):
    """metadataと本体の総数・topic別件数を比較し、読取不能も失敗として返す。"""
    directory = Path(directory)
    result = {'verified': False, 'expected_total': None, 'actual_total': None,
              'topic_differences': {}, 'error': ''}
    try:
        info = yaml.safe_load((directory/'metadata.yaml').read_text())['rosbag2_bagfile_information']
        expected_total = int(info['message_count'])
        expected = Counter({entry['topic_metadata']['name']: int(entry['message_count'])
                            for entry in info['topics_with_message_count']})
        files = info['relative_file_paths']
        if not files or expected_total < 0 or any(count < 0 for count in expected.values()):
            raise ValueError('metadataのファイル一覧または件数が不正です')
        actual = Counter()
        visited = set()
        for name in files:
            path = (directory/name).resolve()
            path.relative_to(directory.resolve())
            if path in visited or not path.is_file():
                raise ValueError('分割ファイルの不足・重複: '+str(name))
            visited.add(path)
            if info['storage_identifier'] == 'mcap':
                from mcap.reader import make_reader
                with path.open('rb') as stream:
                    # 記録順sort不要。CRC検査も有効にし、payloadをdecodeせず数える。
                    for _, channel, _ in make_reader(stream, validate_crcs=True).iter_messages(log_time_order=False):
                        actual[channel.topic] += 1
            elif info['storage_identifier'] == 'sqlite3':
                with closing(sqlite3.connect(path.as_uri()+'?mode=ro', uri=True)) as database:
                    if database.execute('PRAGMA quick_check').fetchone()[0] != 'ok':
                        raise ValueError('SQLite quick_checkに失敗しました')
                    file_count = 0
                    for topic, count in database.execute(
                            'SELECT topics.name, COUNT(*) FROM messages JOIN topics '
                            'ON messages.topic_id=topics.id GROUP BY topics.name'):
                        actual[topic] += count
                        file_count += count
                    if file_count != database.execute('SELECT COUNT(*) FROM messages').fetchone()[0]:
                        raise ValueError('topic情報のないSQLite messageがあります')
            else:
                raise ValueError('未対応のstorage形式: '+str(info['storage_identifier']))
        differences = {topic: {'expected': expected[topic], 'actual': actual[topic]}
                       for topic in sorted(set(expected)|set(actual)) if expected[topic] != actual[topic]}
        result.update(expected_total=expected_total, actual_total=sum(actual.values()),
                      topic_differences=differences)
        result['verified'] = (not differences and expected_total == sum(expected.values())
                              and expected_total == sum(actual.values()))
        if not result['verified']:
            result['error'] = 'metadataと実message件数が一致しません'
    except Exception as error:
        # 未対応依存・破損・permissionを0件の正常bagと誤認しない。
        result['error'] = str(error)
    return result


def main():
    """既存bagも再検査できるCLI。元bagは変更せず、失敗時は非zeroで終了する。"""
    import argparse
    import json
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, help='検査JSONの新規保存先（任意）')
    args = parser.parse_args()
    if args.output and args.output.exists():
        parser.error('既存の検査結果は上書きしません')
    result = verify_message_counts(args.bag)
    data = json.dumps(result, ensure_ascii=False, indent=2)+'\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(data, encoding='utf-8')
    print(data, end='')
    if not result['verified']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
