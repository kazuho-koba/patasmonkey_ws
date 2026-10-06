"""実カメラなしで座標対応・欠測・ground採用点・設定復元を確認する。"""
from types import SimpleNamespace
import json
import numpy as np
import pytest
from pm_evaluation.confidence_analysis import (
    image_array, mapped_confidence, histogram_summary, accepted_ground_pixels,
)


def geometry():
    """人工的な同一cameraを作り、identityなら同一pixelへ戻ることを確認する。"""
    return dict(depth_rgb_K=np.eye(3).tolist(), rgb={'D': [0.]*5},
                right={'K': np.eye(3).tolist()}, rgb_to_right_cm=np.eye(4).tolist(),
                right_rectification=np.eye(3).tolist())


def test_padding_endian():
    message = SimpleNamespace(encoding='16UC1', is_bigendian=True, width=2, height=1,
                              step=6, data=np.array([1000, 2000, 99], dtype='>u2').tobytes())
    assert image_array(message).tolist() == [[1000, 2000]]


def test_identity_missing_and_threshold():
    confidence = np.array([[0, 200, 240], [30, 220, 255]], dtype=np.uint8)
    depth = np.full((2, 3), 1000, dtype=np.uint16); depth[1, 2] = 0
    mapped = mapped_confidence(depth, confidence, geometry())
    assert mapped.tolist() == [[0, 200, 240], [30, 220, -1]]
    assert np.count_nonzero(mapped > 200) == 2


def test_centimetres_and_out_of_fov():
    g = geometry(); g['rgb_to_right_cm'][0][3] = 100.
    depth = np.array([[1000, 1000]], dtype=np.uint16)
    assert mapped_confidence(depth, np.array([[10, 20]], dtype=np.uint8), g).tolist() == [[20, -1]]


def test_ground_dedup_and_histogram(tmp_path):
    path = tmp_path/'cells.csv'
    path.write_text('last_accepted_ground_input_stamp_ns,last_accepted_ground_input_pixel_u,'
                    'last_accepted_ground_input_pixel_v\n10,2,3\n10,2,3\n,,\n')
    assert accepted_ground_pixels(path) == {10: {(2, 3)}}
    h = np.zeros(256, dtype=int); h[0] = 1; h[255] = 3
    summary = histogram_summary(h)
    assert summary['count'] == 4 and summary['mean'] == 191.25 and summary['p95'] == 255
    assert histogram_summary(np.zeros(256))['count'] == 0


def test_configuration_roundtrip_without_device():
    dai = pytest.importorskip('depthai')
    from depthai_driver.confidence_recording import configuration_dict
    from pm_evaluation.cli.reprocess_depth_confidence import restore_configuration, make_pipeline
    pipeline = dai.Pipeline(); stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB); stereo.setOutputSize(640, 400)
    stereo.initialConfig.setConfidenceThreshold(240)
    config = configuration_dict(stereo.initialConfig.get())
    json.dumps(config)
    other = dai.StereoDepthConfig().get(); restore_configuration(other, config)
    assert configuration_dict(other) == config
    calibration = dai.CalibrationHandler()
    snapshot = dict(depthai_version=dai.__version__, eeprom=calibration.eepromToJson(),
                    initial_config=config, out_config=None, pipeline=pipeline.serializeToJson())
    replay = make_pipeline(dai, snapshot, 200)
    props = next(node['properties'] for _, node in replay.serializeToJson()['pipeline']['nodes']
                 if node['name'] == 'StereoDepth')
    assert props['initialConfig']['costMatching']['confidenceThreshold'] == 200
    assert props['outWidth'] == 640 and props['depthAlignCamera'] == int(dai.CameraBoardSocket.RGB)


def test_driver_sequence_pairing_without_camera():
    """Device/Nodeを生成せず、後着診断と既存depthのsequence対応だけを実行する。"""
    from depthai_driver.oakd_vio_rgbd_node import OakdVioRgbdNode
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header
    from builtin_interfaces.msg import Time
    messages = []
    publisher = SimpleNamespace(publish=messages.append)
    packet = SimpleNamespace(getFrame=lambda: np.array([[240]], dtype=np.uint8))
    fake = SimpleNamespace(confidence_frames={7: packet}, disparity_frames={},
                           diagnostic_depth_headers={7: Header(stamp=Time(sec=123))},
                           pub_confidence=publisher, pub_disparity=publisher,
                           pub_confidence_metadata=publisher, pub_disparity_metadata=publisher,
                           rgb_optical_frame='rgb_camera_optical_frame',
                           bridge=SimpleNamespace(cv2_to_imgmsg=lambda data, encoding: Image(
                               width=1, height=1, encoding=encoding, step=1, data=data.tobytes())),
                           make_frame_metadata=lambda *args: 'metadata')
    OakdVioRgbdNode.publish_depth_diagnostics(fake)
    assert not messages and 7 in fake.diagnostic_depth_headers
    fake.disparity_frames[7] = packet
    OakdVioRgbdNode.publish_depth_diagnostics(fake)
    assert len(messages) == 4 and messages[0].header.stamp.sec == 123
    assert messages[0].header.frame_id != messages[2].header.frame_id
    assert not fake.diagnostic_depth_headers


@pytest.mark.parametrize('storage', ['sqlite3', 'mcap'])
def test_bag_analysis_without_device(tmp_path, monkeypatch, storage):
    """実CDRのSQLite/MCAP bagでsnapshot・sequence join・ground分布・maskを確認する。"""
    import sys
    import sqlite3
    import yaml
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image
    from std_msgs.msg import String, Header
    from builtin_interfaces.msg import Time
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from pm_evaluation.cli import analyze_depth_confidence as analysis
    bag = tmp_path/'bag'; bag.mkdir()
    writer = sqlite3.connect(str(bag/'test.db3'))
    writer.execute('CREATE TABLE topics (id INTEGER PRIMARY KEY,name TEXT,type TEXT,serialization_format TEXT)')
    writer.execute('CREATE TABLE messages (id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB)')
    snapshot = dict(schema_version=1, mx_id='synthetic', depthai_version='test',
                    parameters={'confidence_threshold': 240}, eeprom={}, geometry=geometry())
    header = Header(stamp=Time(sec=10))
    samples = {
        analysis.SNAPSHOT: String(data=json.dumps(snapshot)),
        analysis.DEPTH: Image(header=header, width=2, height=2, encoding='16UC1', step=4,
                              data=np.array([[1000, 1000], [1000, 0]], dtype='<u2').tobytes()),
        analysis.CONFIDENCE: Image(header=header, width=2, height=2, encoding='mono8', step=2,
                                   data=bytes([20, 250, 100, 255])),
    }
    for topic in ('/oak/diagnostics/depth_frame', '/oak/diagnostics/confidence_frame'):
        samples[topic] = DiagnosticArray(header=header, status=[DiagnosticStatus(
            values=[KeyValue(key='sequence_num', value='42')])])
    types = {String: 'std_msgs/msg/String', Image: 'sensor_msgs/msg/Image',
             DiagnosticArray: 'diagnostic_msgs/msg/DiagnosticArray'}
    for index, (topic, sample) in enumerate(samples.items()):
        writer.execute('INSERT INTO topics VALUES (?,?,?,?)', (index+1, topic, types[type(sample)], 'cdr'))
        writer.execute('INSERT INTO messages VALUES (?,?,?,?)',
                       (index+1, index+1, 10**10+index, serialize_message(sample)))
    writer.commit(); writer.close()
    filename = 'test.db3'
    if storage == 'mcap':
        from mcap.writer import Writer
        filename = 'test.mcap'
        with (bag/filename).open('wb') as stream:
            writer = Writer(stream); writer.start()
            for index, (topic, sample) in enumerate(samples.items()):
                schema = writer.register_schema(types[type(sample)], 'ros2msg', b'')
                channel = writer.register_channel(topic, 'cdr', schema)
                writer.add_message(channel, 10**10+index, serialize_message(sample), 10**10+index)
            writer.finish()
    (bag/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': {
        'storage_identifier': storage, 'relative_file_paths': [filename]}}))
    ground = tmp_path/'cells.csv'
    ground.write_text('last_accepted_ground_input_stamp_ns,last_accepted_ground_input_pixel_u,'
                      'last_accepted_ground_input_pixel_v\n10000000000,0,0\n10000000000,0,0\n')
    output = tmp_path/'result'
    monkeypatch.setattr(sys, 'argv', ['analyze_depth_confidence', str(bag), '--output', str(output),
                        '--ground-cells', str(ground), '--allow-unvalidated-geometry',
                        '--save-masked-depth', '--thresholds', '200'])
    analysis.main()
    summary = json.loads((output/'summary.json').read_text())
    assert summary['counters']['paired_frames'] == 1
    assert summary['distributions']['accepted_ground_mapped']['mean'] == 20
    assert summary['distributions']['accepted_ground_mapped']['count'] == 1
    masked = np.load(output/'approx_threshold_200/10000000000.npz')
    assert masked['depth_mm'].tolist() == [[1000, 0], [1000, 0]]
