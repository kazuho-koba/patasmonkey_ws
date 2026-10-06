"""保存した左右画像を同じDepthAI設定へ入力し、閾値変更後のdepthを再生成する。

実機の接続は--allow-deviceで明示許可した場合だけ。校正はbagのJSONをpipelineへ
渡し、EEPROMを書換えない。左右からCPUでdepthを再実装せずOAKのStereoDepthを使う。
元閾値でのbaseline一致検証を先に実施し、差があるまま完全再現と呼ばない。
"""
import argparse
import csv
import json
import time
import hashlib
from collections import OrderedDict
from datetime import timedelta
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticArray
from pm_evaluation.confidence_analysis import image_array
from pm_evaluation.cli.analyze_depth_confidence import records, load_snapshot, stamp, DEPTH


def restore_configuration(target, values):
    """記録したpybind設定を復元し、未対応field/型を黙って無視しない。"""
    for name, value in values.items():
        if name == 'unserialized_type' or not hasattr(target, name):
            raise ValueError('復元できない設定: '+name)
        old = getattr(target, name)
        if isinstance(value, dict) and 'name' in value and 'value' in value:
            setattr(target, name, type(old)(value['value']))
        elif isinstance(value, dict):
            restore_configuration(old, value)
        elif isinstance(value, list) and value and isinstance(value[0], dict):
            # filter順序などの固定長enum配列をSDK型へ戻す。文字列/dictのまま渡さない。
            setattr(target, name, [type(old[index])(item['value'])
                                  for index, item in enumerate(value)])
        else:
            setattr(target, name, value)


def make_pipeline(dai, snapshot, threshold):
    """保存校正・初期設定・align条件を復元する。新しいSDKでの代用は拒否する。"""
    if dai.__version__ != snapshot['depthai_version']:
        raise ValueError('記録と同じDepthAI SDK versionが必要です')
    commit = snapshot.get('depthai_commit', 'unavailable')
    if commit != 'unavailable' and str(getattr(dai, '__commit__', 'unavailable')) != commit:
        raise ValueError('DepthAI SDK commitが記録と一致しません')
    pipeline = dai.Pipeline()
    pipeline.setCalibrationData(dai.CalibrationHandler.fromJson(snapshot['eeprom']))
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    raw = stereo.initialConfig.get()
    restore_configuration(raw, snapshot['out_config'] or snapshot['initial_config'])
    stereo.initialConfig.set(raw)
    stereo.initialConfig.setConfidenceThreshold(threshold)
    props = next(node['properties'] for _, node in snapshot['pipeline']['pipeline']['nodes']
                 if node['name'] == 'StereoDepth')
    # pipelineのnode propertiesはconfigとは別。resize/rectification/仕様baseline選択も復元する。
    setters = {
        'alphaScaling': 'setAlphaScaling', 'baseline': 'setBaseline', 'focalLength': 'setFocalLength',
        'depthAlignmentUseSpecTranslation': 'setDepthAlignmentUseSpecTranslation',
        'disparityToDepthUseSpecTranslation': 'setDisparityToDepthUseSpecTranslation',
        'rectificationUseSpecTranslation': 'setRectificationUseSpecTranslation',
        'enableRectification': 'setRectification', 'rectifyEdgeFillColor': 'setRectifyEdgeFillColor',
        'useHomographyRectification': 'useHomographyRectification',
    }
    for name, setter in setters.items():
        if props.get(name) is not None:
            getattr(stereo, setter)(props[name])
    stereo.setDepthAlign(dai.CameraBoardSocket(props['depthAlignCamera']))
    stereo.setOutputSize(props['outWidth'], props['outHeight'])
    stereo.setOutputKeepAspectRatio(props['outKeepAspectRatio'])
    stereo.setInputResolution(640, 400)
    for name, output, socket in (('left', stereo.left, dai.CameraBoardSocket.LEFT),
                                 ('right', stereo.right, dai.CameraBoardSocket.RIGHT)):
        link = pipeline.createXLinkIn(); link.setStreamName(name); link.out.link(output)
    output = pipeline.createXLinkOut(); output.setStreamName('depth'); stereo.depth.link(output.input)
    return pipeline


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--threshold', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--allow-device', action='store_true')
    parser.add_argument('--baseline-report', type=Path,
                        help='記録時閾値で再処理したsummary.json。厳格閾値では必須')
    parser.add_argument('--allow-baseline-difference', action='store_true',
                        help='baselineが完全一致しなくても、差を明記して比較を続ける')
    args = parser.parse_args()
    if not args.allow_device:
        parser.error('OAKを使用するには--allow-deviceが必要です。未接続時には分布解析を使ってください')
    if not 0 <= args.threshold <= 255:
        parser.error('thresholdは0..255')
    snapshot = load_snapshot(args.bag)
    original_threshold = snapshot['parameters']['confidence_threshold']
    if args.threshold > original_threshold:
        parser.error('この検証ツールは記録時と同等／より厳しい閾値だけを対象とします')
    snapshot_hash = hashlib.sha256(json.dumps(snapshot, sort_keys=True).encode()).hexdigest()
    if args.threshold != original_threshold:
        if args.baseline_report is None:
            parser.error('先に記録時閾値のbaselineを生成し、--baseline-reportで指定してください')
        baseline = json.loads(args.baseline_report.read_text())
        if not baseline.get('baseline') or baseline.get('snapshot_sha256') != snapshot_hash or baseline.get('bag') != str(args.bag.resolve()):
            parser.error('baselineのbag／校正設定identityが一致しません')
        if not baseline.get('baseline_exact') and not args.allow_baseline_difference:
            parser.error('baselineに差があります。原因確認後に必要なら--allow-baseline-differenceを明示してください')
    import depthai as dai
    pipeline = make_pipeline(dai, snapshot, args.threshold)
    args.output.mkdir(parents=True, exist_ok=False)
    # 元depthの採用sequenceをmetadataから取得。raw入力は20Hzの全左右ペアを供給し、
    # depthの10Hz採用frameだけ保存する。時間filterの状態を10Hz入力に変えない。
    targets = {}
    for _, data, _ in records(args.bag, ['/oak/diagnostics/depth_frame']):
        message = deserialize_message(data, DiagnosticArray)
        if message.status:
            values = {v.key: v.value for v in message.status[0].values}
            sequence = int(values['sequence_num'])
            if sequence in targets and targets[sequence] != stamp(message):
                raise ValueError('device sequenceが再利用されています。camera再起動区間を分けてください')
            targets[sequence] = stamp(message)
    source_to_target = {}
    for _, data, _ in records(args.bag, ['/oak/diagnostics/left_frame']):
        message = deserialize_message(data, DiagnosticArray)
        if message.status:
            values = {v.key: v.value for v in message.status[0].values}
            sequence = int(values['sequence_num'])
            if sequence in targets:
                source_to_target[stamp(message)] = targets[sequence]
    if not source_to_target:
        raise ValueError('左右／depthのsequence metadataによる対応がありません')
    # baseline比較用depthは必要時にstreamで読む。bag長に比例した全画像保存はしない。
    topics = ['/oak/stereo/left/image_raw', '/oak/stereo/right/image_raw']
    cache = OrderedDict(); produced = 0; lost_pairs = 0
    with dai.Device(pipeline, dai.DeviceInfo(snapshot['mx_id'])) as device:
        queues = {name: device.getInputQueue(name) for name in ('left', 'right')}
        output = device.getOutputQueue('depth', maxSize=4, blocking=False)
        with (args.output/'frames.csv').open('w', newline='') as stream:
            writer = csv.writer(stream); writer.writerow(['stamp_ns', 'sequence', 'valid_pixels'])
            for topic, data, _ in records(args.bag, topics):
                message = deserialize_message(data, Image); key = stamp(message)
                entry = cache.setdefault(key, {})
                entry['left' if '/left/' in topic else 'right'] = message
                if len(entry) != 2:
                    while len(cache) > 40:
                        cache.popitem(last=False); lost_pairs += 1
                    continue
                cache.pop(key)
                # 元左右topicはsequence-matchedで共通stamp。hardwareへの再入力sequenceは
                # 独立連番とし、出力との照合を厳密にする。
                sequence = produced; produced += 1
                for name, source in entry.items():
                    frame = dai.ImgFrame(); frame.setType(dai.ImgFrame.Type.RAW8)
                    frame.setWidth(source.width); frame.setHeight(source.height)
                    frame.setInstanceNum(int(dai.CameraBoardSocket.LEFT if name == 'left'
                                             else dai.CameraBoardSocket.RIGHT))
                    frame.setSequenceNum(sequence)
                    frame.setTimestamp(timedelta(seconds=key*1e-9))
                    frame.setData(np.ascontiguousarray(image_array(source)).ravel())
                    queues[name].send(frame)
                deadline = time.monotonic()+10.0; result = None
                while time.monotonic() < deadline:
                    result = output.tryGet()
                    if result is not None:
                        break
                    time.sleep(.002)
                if result is None or result.getSequenceNum() != sequence:
                    raise RuntimeError('StereoDepthの出力欠落／sequence不一致')
                # 同じdevice sequenceのdepth stampへ戻す。stereo timestampとdepth timestampの
                # 微小差を最近傍joinで埋めず、非採用20Hz frameは状態更新にだけ使う。
                image = result.getFrame()
                if key in source_to_target:
                    target_stamp = source_to_target[key]
                    np.save(args.output/(str(target_stamp)+'.npy'), image)
                    writer.writerow([target_stamp, sequence, int(np.count_nonzero(image))])
    comparisons = []
    for _, data, _ in records(args.bag, [DEPTH]):
        message = deserialize_message(data, Image); key = stamp(message)
        path = args.output/(str(key)+'.npy')
        if not path.exists():
            comparisons.append({'stamp_ns': key, 'missing': True}); continue
        generated, reference = np.load(path), image_array(message)
        if generated.shape != reference.shape:
            raise ValueError('再生成depthの寸法不一致')
        common = (generated > 0) & (reference > 0)
        differences = np.abs(generated.astype(float)-reference.astype(float))[common]
        comparisons.append(dict(stamp_ns=key, equal_pixels=int(np.count_nonzero(generated == reference)),
                                pixels=reference.size, valid_mask_difference=int(np.count_nonzero(
                                    (generated > 0) != (reference > 0))),
                                common_valid_pixels=int(common.sum()),
                                mean_abs_difference_mm=float(differences.mean()) if len(differences) else None,
                                max_abs_difference_mm=float(differences.max()) if len(differences) else None))
    summary = dict(threshold=args.threshold, recorded_threshold=original_threshold,
                   bag=str(args.bag.resolve()), snapshot_sha256=snapshot_hash,
                   baseline=args.threshold == original_threshold, replay_pairs=produced,
                   baseline_exact=(args.threshold == original_threshold and bool(comparisons)
                                   and all(not c.get('missing') and c['equal_pixels'] == c['pixels']
                                           for c in comparisons)),
                   unmatched_pair_evictions=lost_pairs, partial_pairs_at_end=len(cache),
                   recorded_depth_sequences=len(targets), comparisons=comparisons,
                   caveats=['bag開始前のtemporal状態は復元不可',
                            '元閾値baselineの一致が未確認なら完全再現と呼ばない'])
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2))
    print('replayed {} stereo pairs; compare summary.json before interpreting stricter threshold'.format(produced))
