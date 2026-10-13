"""診断readerのQoSをROS nodeやハードウェアなしで確認する。"""
import importlib.util
from pathlib import Path
import pytest
from rclpy.qos import ReliabilityPolicy

path = Path(__file__).parents[1]/'tools/probe_imu_delivery.py'
spec = importlib.util.spec_from_file_location('probe_imu_delivery_tested', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_reader_qos_is_explicit_and_keeps_depth():
    for name, policy in [('best_effort', ReliabilityPolicy.BEST_EFFORT),
                         ('reliable', ReliabilityPolicy.RELIABLE)]:
        qos = module.probe_qos(200, name)
        assert qos.depth == 200
        assert qos.reliability == policy


@pytest.mark.parametrize('depth,reliability', [(0, 'reliable'), (200, 'unknown')])
def test_reader_rejects_ambiguous_qos(depth, reliability):
    with pytest.raises(ValueError):
        module.probe_qos(depth, reliability)
