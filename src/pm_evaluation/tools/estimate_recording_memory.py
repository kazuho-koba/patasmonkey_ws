"""実測容量から有限RAM保存とSSD書込み停滞の容量を概算する（ROS不要）。

圧縮後MCAP容量と圧縮前MCAP容量は別入力にする。非同期FIFOは圧縮前CDRを
保持するため、圧縮後のファイル帯域だけでqueueを選ぶと過小評価になる。
MCAPの圧縮前容量にはrecord overheadも含まれるのでCDR量そのものではない。
平均帯域の概算はburst・DDS履歴・圧縮workspace・page cacheを保証しない。
"""
import argparse
import json
import math


def estimate(file_bytes, uncompressed_bytes, duration_s, pause_s, buffer_bytes,
             ram_budget_bytes, safety_factor=1.5, message_count=None, message_limit=8192):
    """容量[byte]と時間[s]から計画値を返す。実際の無欠損は別途検証する。"""
    values = (file_bytes, uncompressed_bytes, duration_s, buffer_bytes,
              ram_budget_bytes, safety_factor)
    if any(not math.isfinite(value) or value <= 0 for value in values):
        raise ValueError('容量・期間・安全係数は有限の正数が必要です')
    if not math.isfinite(pause_s) or pause_s < 0 or safety_factor < 1:
        raise ValueError('停滞時間は非負、安全係数は1以上が必要です')
    disk_rate = file_bytes / duration_s
    raw_rate = uncompressed_bytes / duration_s
    # SSDへ書けない間の新規流入を平均値で見積もり、選択可能な安全係数を掛ける。
    # これは必要容量の計画値であり、観測済み最大pauseが今後の上限ではない。
    result = {
        'compressed_bytes_per_s': disk_rate,
        'uncompressed_mcap_bytes_per_s': raw_rate,
        'pause_buffer_estimate_bytes': math.ceil(raw_rate * pause_s * safety_factor),
        'buffer_average_pause_coverage_s': buffer_bytes / (raw_rate * safety_factor),
        'ram_only_budget_duration_s': ram_budget_bytes / (disk_rate * safety_factor),
        'compressed_bytes_per_hour': disk_rate * 3600,
        'safety_factor': safety_factor,
        # spoolが一定容量に収まるには平均SSD保存能力が流入量を上回る必要がある。
        # OS page cacheへのwrite完了は電源断時の永続化保証とは別である。
        'ssd_sustained_rate_must_exceed_bytes_per_s': disk_rate,
    }
    # FIFOにはtopic登録/削除も入る。message件数からのitem容量は楽観的概算で、
    # 実際の制御item・サイズの偏りを別途確認する。byte上限だけを見ない。
    if message_count is not None:
        if not math.isfinite(message_count) or message_count <= 0 or message_limit <= 0:
            raise ValueError('message件数とitem上限は正数が必要です')
        item_coverage = message_limit * duration_s / (message_count * safety_factor)
        result['item_average_pause_coverage_s'] = item_coverage
        result['combined_average_pause_coverage_s'] = min(
            result['buffer_average_pause_coverage_s'], item_coverage)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file-bytes', required=True, type=int)
    parser.add_argument('--uncompressed-bytes', required=True, type=int)
    parser.add_argument('--duration-s', required=True, type=float)
    parser.add_argument('--pause-s', required=True, type=float)
    parser.add_argument('--buffer-mib', required=True, type=float)
    parser.add_argument('--ram-budget-mib', required=True, type=float)
    parser.add_argument('--safety-factor', type=float, default=1.5)
    parser.add_argument('--message-count', type=int,
                        help='全記録message件数。指定時はitem上限も概算する')
    parser.add_argument('--message-limit', type=int, default=8192)
    args = parser.parse_args()
    try:
        result = estimate(args.file_bytes, args.uncompressed_bytes, args.duration_s,
                          args.pause_s, args.buffer_mib * 2**20,
                          args.ram_budget_mib * 2**20, args.safety_factor,
                          args.message_count, args.message_limit)
    except ValueError as error:
        parser.error(str(error))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
