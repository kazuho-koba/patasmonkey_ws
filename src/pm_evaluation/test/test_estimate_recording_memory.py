"""圧縮後容量とFIFO容量を混同せず、容量見積りの境界を確認する。"""
import importlib.util
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location(
    'estimate_recording_memory', Path(__file__).parents[1] /
    'tools/estimate_recording_memory.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_raw_queue_and_compressed_ram_are_distinct():
    result = module.estimate(1000, 2000, 10, 3, 900, 1500, 1.5)
    assert result['pause_buffer_estimate_bytes'] == 900
    assert result['buffer_average_pause_coverage_s'] == 3
    assert result['ram_only_budget_duration_s'] == 10
    assert result['compressed_bytes_per_hour'] == 360000


def test_item_limit_can_dominate_large_byte_budget():
    result = module.estimate(1000, 2000, 10, 3, 9000, 1500, 1.5, 100, 15)
    assert result['item_average_pause_coverage_s'] == 1
    assert result['combined_average_pause_coverage_s'] == 1


@pytest.mark.parametrize('duration,pause,factor', [(0, 1, 1), (1, -1, 1),
                                                  (1, 1, .9), (float('nan'), 1, 1)])
def test_invalid_planning_values(duration, pause, factor):
    with pytest.raises(ValueError):
        module.estimate(1000, 2000, duration, pause, 900, 1500, factor)
