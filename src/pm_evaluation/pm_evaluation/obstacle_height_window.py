"""障害物候補を即時保持し、有効な短期再観測で解除する診断用状態。

高さはground基準のm。これはfree-ray判定ではなく、低い再観測による
解除仮説を調べるための実験であり、実機の安全出力に直接使わない。
"""
from collections import deque
import math


class ObstacleHeightWindow:
    """未観測は中立。新hitは即時黒、解除にはN個の有効観測を要求する。"""

    def __init__(self, window, limit):
        if window not in (1, 3, 5) or not math.isfinite(limit) or limit <= 0:
            raise ValueError('Nは1/3/5、limitは有限の正値です')
        self.window, self.limit = window, limit
        self.history = deque(maxlen=window)
        self.active = self.confirmed = False
        self.onset = None
        self.value = float('nan')

    def update(self, height, stamp, confirmation=False):
        """今回の高さと同群支持を反映し、解除前の状態名を返す。

        historyは有効なセル観測のみ。新規黒の開始時に以前の低い値を捨て、
        その後は直近N観測の算術平均を使う。画素数で重み付けしない。
        RVizの整数cost丸めで100になる境界（limitの99.5%）も黒として保持。
        """
        if not math.isfinite(height):
            return None
        height = max(0., height)
        hit = height >= self.limit
        if hit and not self.active:
            self.active, self.confirmed, self.onset = True, False, stamp
            self.history.clear()
        self.history.append(height)
        mean = sum(self.history) / len(self.history)
        if self.active:
            self.confirmed |= bool(confirmation) or (hit and self.window == 1)
            if not hit and len(self.history) == self.window and mean < .995*self.limit:
                previous = 'confirmed' if self.confirmed else 'pending'
                self.active = self.confirmed = False
                self.value = mean
                return previous
            # 確認待ちも保持。窓が未充足でも新しい危険を薄めない。
            self.value = max(mean, self.limit)
        else:
            self.value = mean
        return None
