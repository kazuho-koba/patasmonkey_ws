"""USB・ROSなしで補助feedbackの更新頻度と診断保存を検証する。"""
import csv
import pytest
from pm_control.motor_polling import FeedbackCache, PollingTiming


def test_default_reads_every_call():
    cache = FeedbackCache()
    calls = []
    for now in [0, .001, .002]:
        _, refreshed = cache.get(lambda: calls.append(now) or (1., 2., 3.), now)
        assert refreshed
    assert len(calls) == 3


def test_low_rate_cache_and_reset():
    cache = FeedbackCache(10)
    assert cache.get(lambda: (1., 2., 3.), 0) == ((1., 2., 3.), True)
    assert cache.get(lambda: pytest.fail('まだ読み取らない'), .05) == ((1., 2., 3.), False)
    assert cache.get(lambda: (4., 5., 6.), .11) == ((4., 5., 6.), True)
    cache.reset()
    assert cache.get(lambda: (7., 8., 9.), .12)[1]


def test_read_failure_does_not_replace_cache():
    cache = FeedbackCache(10)
    cache.get(lambda: (1., 2., 3.), 0)
    def broken():
        raise RuntimeError('USB error')
    with pytest.raises(RuntimeError):
        cache.get(broken, .11)
    assert cache.value == (1., 2., 3.)
    assert cache.last_read == 0


@pytest.mark.parametrize('hz', [0, -1, 31, float('nan'), float('inf')])
def test_invalid_rate(hz):
    with pytest.raises(ValueError):
        FeedbackCache(hz)


def test_timing_is_bounded_and_exclusive(tmp_path):
    timing = PollingTiming(tmp_path/'timing.csv', capacity=2)
    for i in range(3):
        timing.add(i, 1, 1.002, 1.006, 1.007, 1.008, True, 'ok')
    timing.save()
    with timing.path.open() as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 2
    assert rows[0]['sequence'] == '1'
    assert float(rows[0]['total_ms']) == pytest.approx(8)
    with pytest.raises(FileExistsError):
        timing.save()
