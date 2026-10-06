"""mission bagのconfidence分布・採用ground画素をカメラなしで解析する。"""
import argparse
import csv
import json
from collections import OrderedDict
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
from pm_evaluation.confidence_analysis import (
    accepted_ground_pixels, histogram_summary, image_array, mapped_confidence,
)

SNAPSHOT = '/oak/stereo/recording_snapshot'
CONFIDENCE = '/oak/stereo/confidence/image_raw'
DEPTH = '/oak/depth/image_raw'


def records(bag, topics):
    """MCAP/SQLiteのCDRをstreamする。Foxyで未提供の場合があるrosbag2_pyは不要。"""
    import sqlite3
    import yaml
    metadata = yaml.safe_load((Path(bag)/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    for relative in metadata['relative_file_paths']:
        path = Path(bag)/relative
        if metadata['storage_identifier'] == 'mcap':
            from mcap.reader import make_reader
            with path.open('rb') as stream:
                for _, channel, record in make_reader(stream).iter_messages(topics=topics):
                    yield channel.topic, record.data, record.log_time
        elif metadata['storage_identifier'] == 'sqlite3':
            connection = sqlite3.connect('file:'+str(path.resolve())+'?mode=ro', uri=True)
            try:
                placeholders = ','.join('?' for _ in topics)
                query = ('SELECT topics.name,messages.data,messages.timestamp FROM messages '
                         'JOIN topics ON messages.topic_id=topics.id WHERE topics.name IN ('+
                         placeholders+') ORDER BY messages.timestamp,messages.id')
                yield from connection.execute(query, tuple(topics))
            finally:
                connection.close()
        else:
            raise ValueError('未対応bag storage: '+metadata['storage_identifier'])


def stamp(message):
    """撮像stampを整数nsでjoinする。記録時刻の近い別frameを代用しない。"""
    return message.header.stamp.sec*10**9+message.header.stamp.nanosec


def load_snapshot(bag):
    """後着した実効outConfigを優先し、異なるcamera/設定の混在を拒否する。"""
    chosen = None; identity = None
    for _, data, _ in records(bag, [SNAPSHOT]):
        snapshot = json.loads(deserialize_message(data, String).data)
        current = (snapshot['mx_id'], snapshot['depthai_version'],
                   json.dumps(snapshot['parameters'], sort_keys=True),
                   json.dumps(snapshot['eeprom'], sort_keys=True))
        if identity is not None and identity != current:
            raise ValueError('複数camera／設定／校正のbagは区間ごとに分けてください')
        identity = current
        if chosen is None or snapshot.get('out_config') is not None:
            chosen = snapshot
    if chosen is None:
        raise ValueError('校正snapshot未記録。現在の実機校正で古いbagを補完しません')
    return chosen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ground-cells', type=Path, help='mapper forensicのcells.csv')
    parser.add_argument('--thresholds', type=int, nargs='+', default=[200, 220, 240])
    parser.add_argument('--allow-unvalidated-geometry', action='store_true',
                        help='実機未検証のRGB→confidence対応を近似診断として許可')
    parser.add_argument('--depth-pixels-rectified', action='store_true',
                        help='depthのRGB画素を歪みなしとして扱う（実機warp確認後に選択）')
    parser.add_argument('--save-masked-depth', action='store_true')
    args = parser.parse_args()
    if any(not 0 <= v <= 255 for v in args.thresholds):
        parser.error('thresholdは0..255')
    if (args.ground_cells or args.save_masked_depth) and not args.allow_unvalidated_geometry:
        parser.error('ground対応／maskは--allow-unvalidated-geometryを明示してください')
    snapshot = load_snapshot(args.bag)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'oak_stereo_snapshot.json').write_text(json.dumps(snapshot, indent=2))
    ground = accepted_ground_pixels(args.ground_cells) if args.ground_cells else {}
    histograms = {name: np.zeros(256, dtype=np.int64) for name in
                  ('confidence_native_all', 'valid_depth_mapped', 'accepted_ground_mapped')}
    cache = OrderedDict()
    counters = dict(paired_frames=0, unpaired_frames=0, valid_depth_pixels=0,
                    mapped_depth_pixels=0, ground_requested=sum(map(len, ground.values())),
                    ground_mapped=0, metadata_missing=0, metadata_sequence_mismatch=0)
    ground_stream = (args.output/'accepted_ground_confidence.csv').open('w', newline='')
    ground_writer = csv.writer(ground_stream)
    ground_writer.writerow(['source_stamp_ns', 'pixel_u', 'pixel_v', 'axial_depth_mm',
                            'confidence', 'correspondence_valid', 'geometry_validated'])
    requested = [DEPTH, CONFIDENCE, '/oak/diagnostics/depth_frame', '/oak/diagnostics/confidence_frame']
    with (args.output/'frames.csv').open('w', newline='') as stream, ground_stream:
        writer = csv.DictWriter(stream, fieldnames=['stamp_ns', 'valid_pixels', 'mapped_pixels']+
                                ['removed_gt_'+str(t) for t in args.thresholds])
        writer.writeheader()
        def process(key, entry):
            if not {'depth', 'confidence'}.issubset(entry):
                counters['unpaired_frames'] += 1; return
            if 'depth_meta' in entry and 'confidence_meta' in entry:
                if entry['depth_meta'] != entry['confidence_meta']:
                    counters['metadata_sequence_mismatch'] += 1; return
            else:
                # 対応確認を省略して別画像を混ぜない。欠測は分布にも入れない。
                counters['metadata_missing'] += 1; return
            depth, confidence = entry['depth'], entry['confidence']
            if confidence.dtype != np.uint8:
                raise ValueError('confidenceは8bit画像が必要です')
            counters['paired_frames'] += 1
            histograms['confidence_native_all'] += np.bincount(confidence.ravel(), minlength=256)
            valid = int(np.count_nonzero(depth)); counters['valid_depth_pixels'] += valid
            if not args.allow_unvalidated_geometry:
                writer.writerow({'stamp_ns': key, 'valid_pixels': valid}); return
            mapping = mapped_confidence(depth, confidence, snapshot['geometry'], args.depth_pixels_rectified)
            mask = mapping >= 0
            matched = int(mask.sum()); counters['mapped_depth_pixels'] += matched
            histograms['valid_depth_mapped'] += np.bincount(mapping[mask], minlength=256)
            selected = []
            for u, v in ground.get(key, ()):
                if 0 <= v < depth.shape[0] and 0 <= u < depth.shape[1] and mapping[v, u] >= 0:
                    selected.append(mapping[v, u])
                    ground_writer.writerow([key, u, v, int(depth[v, u]), int(mapping[v, u]), True, False])
                else:
                    ground_writer.writerow([key, u, v, '', '', False, False])
            counters['ground_mapped'] += len(selected)
            if selected:
                histograms['accepted_ground_mapped'] += np.bincount(selected, minlength=256)
            row = {'stamp_ns': key, 'valid_pixels': valid, 'mapped_pixels': matched}
            for threshold in args.thresholds:
                remove = mask & (mapping > threshold)
                row['removed_gt_'+str(threshold)] = int(remove.sum())
                if args.save_masked_depth:
                    directory = args.output/('approx_threshold_'+str(threshold))
                    directory.mkdir(exist_ok=True)
                    image = depth.copy(); image[remove] = 0
                    # 対応不能画素は元depthを保持し、別maskで表示。厳格条件確認済みとは数えない。
                    np.savez_compressed(directory/(str(key)+'.npz'), depth_mm=image,
                                        correspondence_valid=mask)
            writer.writerow(row)
        for topic, data, _ in records(args.bag, requested):
            is_image = topic in (DEPTH, CONFIDENCE)
            message = deserialize_message(data, Image if is_image else DiagnosticArray)
            key = stamp(message); entry = cache.setdefault(key, {})
            if is_image:
                entry['depth' if topic == DEPTH else 'confidence'] = image_array(message).copy()
            elif message.status:
                values = {v.key: v.value for v in message.status[0].values}
                entry['depth_meta' if topic.endswith('/depth_frame') else 'confidence_meta'] = int(values['sequence_num'])
            # 40frameのbounded cacheで後着metadataを待つ。メモリ量はbag長に比例させない。
            while len(cache) > 40:
                old_key, old = cache.popitem(last=False); process(old_key, old)
        for key, entry in cache.items():
            process(key, entry)
    summary = dict(counters=counters, geometry_validated=False,
                   geometry_mapping_enabled=args.allow_unvalidated_geometry,
                   threshold_comparison='additional mask, not onboard pipeline reproduction',
                   ground_scope='deduplicated accepted source pixels in supplied forensic ROI',
                   distributions={k: histogram_summary(v) for k, v in histograms.items()})
    summary['thresholds'] = {
        str(t): {name: {'count_gt_threshold': int(h[t+1:].sum()),
                        'fraction_gt_threshold': float(h[t+1:].sum()/h.sum()) if h.sum() else None}
                 for name, h in histograms.items()} for t in args.thresholds}
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2))
    with (args.output/'histograms.csv').open('w', newline='') as stream:
        writer = csv.writer(stream); writer.writerow(['confidence']+list(histograms))
        writer.writerows([i]+[int(h[i]) for h in histograms.values()] for i in range(256))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for name, histogram in histograms.items():
        if histogram.sum():
            plt.plot(np.arange(256), histogram/histogram.sum(), label=name)
    plt.xlabel('confidence (0=high confidence)'); plt.ylabel('pixel fraction')
    if any(h.sum() for h in histograms.values()):
        plt.legend()
    plt.tight_layout(); plt.savefig(str(args.output/'confidence_histograms.png')); plt.close()
    print(json.dumps({**counters, 'output': str(args.output)}, indent=2))
