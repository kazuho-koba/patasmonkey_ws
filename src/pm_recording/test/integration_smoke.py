"""隔離ROS domainで、実storageの分割・停止flush・容量超過を確認する。

センサ／車両nodeは起動しない。build後に明示的に実行する統合試験で、
保存先はmkdtempの専用directory。証拠を削除せず最後にpathを表示する。
"""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import random
import string
import csv

import rclpy
from std_msgs.msg import String
from ament_index_python.packages import get_package_prefix
import yaml


def trial(root, label, limit, oversized=False):
    """publisherの接続を待ち、stop後のmetadataとMCAP実件数を照合する。"""
    from pm_bringup.bag_integrity import verify_message_counts
    output = root/label
    binary = Path(get_package_prefix('pm_recording'))/'lib/pm_recording/async_recorder'
    node = rclpy.create_node('async_writer_integration_'+label)
    publisher = node.create_publisher(String, '/async_writer_test', 100)
    with (root/(label+'.log')).open('w') as stream:
        child = subprocess.Popen([str(binary), '-o', str(output), '-s', 'mcap',
                                  '--max-bag-size', '250000', '--queue-bytes', str(limit),
                                  '/async_writer_test'], stdout=stream, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic()+20
            while publisher.get_subscription_count() == 0 and child.poll() is None:
                if time.monotonic() > deadline:
                    raise RuntimeError('discovery timeout')
                rclpy.spin_once(node, timeout_sec=0.05)
            if child.poll() is not None:
                raise RuntimeError('recorder startup failed')
            # 単一文字の反復は圧縮で極端に小さくなり、分割試験にならない。
            # 固定seedの文字列で再現性を保ちつつ、圧縮後も分割閾値を超える量を送る。
            generator = random.Random(0)
            payload = String(data=''.join(generator.choices(string.ascii_letters+string.digits, k=100000)))
            count = 1 if oversized else 30
            for _ in range(count):
                publisher.publish(payload)
                rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.5)
        finally:
            # 容量超過はrecorder自身がshutdown/flushする。途中で二度目のSIGINTを
            # 重ねると終了処理を中断し得るので、この条件では自然終了を待つ。
            if not oversized and child.poll() is None:
                child.send_signal(signal.SIGINT)
            child.wait(timeout=30)
            node.destroy_node()
    stats = yaml.safe_load((output/'async_writer.yaml').read_text())
    with (output/'async_writer.csv').open(newline='') as trace:
        final = list(csv.DictReader(trace))[-1]
    assert int(final['accepted_bytes']) == stats['accepted_bytes']
    assert int(final['written_bytes']) == stats['written_bytes']
    assert int(final['pending_bytes']) == 0
    integrity = verify_message_counts(output)
    assert integrity['verified'], integrity
    if oversized:
        assert child.returncode != 0 and not stats['verified'], stats
        assert stats['error'] and stats['accepted'] == 0, stats
        assert stats['accepted_bytes'] == stats['written_bytes'] == 0
    else:
        assert child.returncode == 0 and stats['verified'], stats
        assert stats['accepted'] == stats['written'] == integrity['actual_total'] == count, stats
        assert stats['accepted_bytes'] == stats['written_bytes'] > 0
        assert len(list(output.glob('*.mcap'))) > 1
        from mcap.reader import make_reader
        from rclpy.serialization import deserialize_message
        restored = 0
        compressions = set()
        for path in output.glob('*.mcap'):
            with path.open('rb') as stream:
                reader = make_reader(stream, validate_crcs=True)
                compressions.update(chunk.compression for chunk in reader.get_summary().chunk_indexes)
                for _, _, message in reader.iter_messages(log_time_order=False):
                    assert deserialize_message(message.data, String).data == payload.data
                    restored += 1
        assert restored == count
        if os.environ.get('PM_MCAP_PRESET_PROFILE') == 'zstd_fast_crc':
            assert compressions == {'zstd'}, compressions
        subprocess.run(['ros2', 'bag', 'info', str(output)], check=True, stdout=subprocess.DEVNULL)
    print(label, stats, integrity, flush=True)


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '91':
        raise RuntimeError('実機と分離するためROS_DOMAIN_ID=91で実行してください')
    root = Path(tempfile.mkdtemp(prefix='pm_async_writer_'))
    rclpy.init(args=[])
    try:
        trial(root, 'split_flush', 64*1024*1024)
        trial(root, 'overflow', 8, oversized=True)
    finally:
        rclpy.shutdown()
        print('ARTIFACT_DIRECTORY', root, flush=True)


if __name__ == '__main__':
    main()
