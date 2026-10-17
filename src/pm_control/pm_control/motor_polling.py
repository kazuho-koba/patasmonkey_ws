"""位置の30Hz読取りを保持したまま、補助telemetryのUSB読取りを間引く。"""
import csv
import math
import time
from collections import deque
from pathlib import Path


class FeedbackCache:
    """速度・電流の一組を保持する。既定30Hzでは従来同様、毎回実読取りする。"""

    def __init__(self, hz=30.0):
        if not math.isfinite(hz) or not 0 < hz <= 30:
            raise ValueError('motor_feedback_hzは0超〜30Hzにしてください')
        self.hz = hz
        self.reset()

    def reset(self):
        """再接続時に前の装置状態を再利用しない。"""
        self.value = None
        self.last_read = None

    def get(self, reader, now=None):
        """reader成功時だけcacheを更新し、値と実読取り有無を返す。"""
        now = time.monotonic() if now is None else now
        refresh = (self.hz == 30 or self.value is None or
                   now - self.last_read >= 1.0 / self.hz)
        if refresh:
            value = reader()
            self.value = value
            self.last_read = now
        return self.value, refresh


class PollingTiming:
    """診断有効時のみ有界メモリに保存し、終了後にCSVを書き出す。"""

    def __init__(self, path, capacity=60000):
        self.path = Path(path)
        self.rows = deque(maxlen=capacity)
        self.total = 0

    def add(self, wall_ns, start, position, feedback, voltage, end, refreshed, status):
        """単調時計による各段階の所要時間。未到達区間は空欄で示す。"""
        def elapsed(a, b):
            return '' if a is None or b is None else (b-a)*1000
        self.rows.append((self.total, wall_ns, start, elapsed(start, position),
                          elapsed(position, feedback), elapsed(feedback, voltage),
                          elapsed(voltage, end), elapsed(start, end), refreshed, status))
        self.total += 1

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open('x', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['sequence', 'wall_ns', 'start_s', 'position_ms', 'feedback_ms',
                             'voltage_ms', 'publish_ms', 'total_ms', 'feedback_refreshed', 'status'])
            writer.writerows(self.rows)
