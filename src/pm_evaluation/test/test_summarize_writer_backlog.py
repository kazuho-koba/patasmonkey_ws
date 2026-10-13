"""保存の追い付きと、診断の欠落を正しく区別する。"""
import importlib.util
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location(
    'summarize_writer_backlog', Path(__file__).parents[1]/'tools/summarize_writer_backlog.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def row(t, accepted, written):
    return dict(elapsed_s=t, accepted_bytes=accepted, written_bytes=written,
                pending_bytes=accepted-written, pending_items=int(accepted != written))


def test_delayed_storage_recovers():
    result = module.summarize([row(0, 0, 0), row(1, 100, 0), row(2, 200, 200)])
    assert result['sampled_peak_pending_bytes'] == 100
    assert result['final_pending_bytes'] == 0
    assert result['intervals'][1]['written_bytes_per_s'] == 200


def test_old_csv_is_not_zero_input():
    with pytest.raises(ValueError):
        module.summarize([dict(elapsed_s=0), dict(elapsed_s=1)])


def test_counter_reset_is_not_a_negative_rate():
    with pytest.raises(ValueError):
        module.summarize([row(0, 100, 100), row(1, 0, 0)])
