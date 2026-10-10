"""停止中の電圧と走行中のDC bus電流から参考残量を計算する。

5直列Li-ionを仮定した汎用の電圧曲線を使う。メーカーの校正表ではなく、
負荷・温度・劣化・配線損失は補償しない。セル単位の安全性も判断しない。
"""

import math
import time


DEFAULT_CURVE = (
    (16.0, 0.0), (16.5, 5.0), (17.5, 10.0), (18.0, 20.0),
    (18.5, 35.0), (19.0, 50.0), (19.5, 65.0), (20.0, 80.0),
    (20.5, 90.0), (21.0, 100.0),
)


class BatteryEstimator:
    """受信callbackだけで更新し、UIへ計算済みの値を渡す。単位はVと%。"""

    def __init__(self, config=None):
        config = config or {}
        self.curve = tuple(tuple(float(v) for v in row)
                           for row in config.get('voltage_soc_curve', DEFAULT_CURVE))
        if (len(self.curve) < 2 or any(len(row) != 2 for row in self.curve)
                or any(not math.isfinite(v) for row in self.curve for v in row)
                or self.curve[0][1] != 0.0 or self.curve[-1][1] != 100.0
                or self.curve[0][0] <= 0.0
                or any(not 0.0 <= row[1] <= 100.0 for row in self.curve)
                or any(b[0] <= a[0] or b[1] < a[1]
                       for a, b in zip(self.curve, self.curve[1:]))):
            raise ValueError('battery.voltage_soc_curveは昇順の[V, %]、端点0/100%が必要です')
        self.low = float(config.get('low_voltage_v', 17.0))
        self.critical = float(config.get('critical_voltage_v', 16.0))
        self.high = float(config.get('high_voltage_v', 21.3))
        self.tau = float(config.get('smoothing_time_constant_sec', 5.0))
        self.rest_settle = float(config.get('rest_settle_sec', 5.0))
        self.stale_timeout = float(config.get('stale_timeout_sec', 3.0))
        self.pack_capacity = float(config.get('capacity_ah_per_pack', 6.0))
        self.parallel_packs = float(config.get('parallel_packs', 2))
        self.capacity = self.pack_capacity*self.parallel_packs
        if (any(not math.isfinite(v) for v in
                (self.low, self.critical, self.high, self.tau, self.stale_timeout,
                 self.rest_settle, self.pack_capacity, self.parallel_packs, self.capacity))
                or not 0.0 < self.critical < self.low < self.high
                or self.tau < 0.0 or self.stale_timeout <= 0.0 or self.rest_settle < 0.0
                or self.pack_capacity <= 0.0 or self.parallel_packs < 1.0
                or not self.parallel_packs.is_integer()):
            raise ValueError('batteryの閾値・時定数・容量・並列個数が不正です')
        self._reset()

    def _reset(self):
        """欠測中の消費や電池交換は推測せず、新たな停止電圧基準を要求する。"""
        self._stamp = None
        self._arrival_stamp = None
        self._clock_source = None
        self._soc = None
        self._anchor_voltage = None
        self._consumed_ah = 0.0
        self._rest_since = None
        self._rest_voltage = None

    def update(self, voltage, ibus_a=None, stationary=False, sample_time=None):
        """停止中はV→SOC、走行中は正のIbusの矩形積分でSOCを減らす。

        sample_timeはMotorState.stampの秒値。bag再生速度に左右されないデータ時刻を
        優先し、stampがない場合だけmonotonic受信時刻を使う。基準時計変更・逆行・
        staleを超える欠測では積算を破棄する。負電流はゼロとして扱い、残量を増やさない。
        5秒停止と電圧平滑化は厳密なOCVやセル安全性を保証するものではない。
        """
        arrival = time.monotonic()
        stamp = arrival if sample_time is None else float(sample_time)
        source = 'receipt' if sample_time is None else 'message'
        voltage = float(voltage)
        current = None if ibus_a is None else float(ibus_a)
        current_valid = current is not None and math.isfinite(current)
        if not math.isfinite(voltage) or voltage <= 0.0 or not math.isfinite(stamp):
            self._reset()
            return self._snapshot(None, current if current_valid else None, 'INVALID', 'WAIT_BASELINE')
        gap = (self._stamp is None or self._clock_source != source
               or stamp < self._stamp or stamp-self._stamp > self.stale_timeout
               or self._arrival_stamp is None or arrival-self._arrival_stamp > self.stale_timeout)
        if gap:
            self._reset()
        dt = 0.0 if self._stamp is None else max(0.0, stamp-self._stamp)
        self._stamp, self._arrival_stamp, self._clock_source = stamp, arrival, source
        if stationary:
            # 走行中の負荷電圧を停止基準へ持ち込まず、停止区間だけを平滑化する。
            if self._rest_since is None:
                self._rest_since, self._rest_voltage = stamp, voltage
            else:
                alpha = 1.0 if self.tau == 0.0 else -math.expm1(-dt/self.tau)
                self._rest_voltage += alpha*(voltage-self._rest_voltage)
            if self._soc is None:
                # 初回の停止中受信は暫定基準。走行中の初回受信からはSOCを作らない。
                self._soc = self._percent(self._rest_voltage)
                self._anchor_voltage = self._rest_voltage
            settled = stamp-self._rest_since >= self.rest_settle
            if settled:
                self._soc = self._percent(self._rest_voltage)
                self._anchor_voltage = self._rest_voltage
                self._consumed_ah = 0.0
            # 整定待ちの5秒間は直前の推定を保持し、その後停止電圧で再校正する。
            mode = 'REST_ESTIMATE' if settled else 'WAIT_SETTLE'
        else:
            self._rest_since = self._rest_voltage = None
            if not current_valid:
                # 有効な電流がない区間をゼロ消費として扱うと残量を過大評価する。
                self._soc = self._anchor_voltage = None
                self._consumed_ah = 0.0
                mode = 'CURRENT_INVALID'
            elif self._soc is None:
                mode = 'WAIT_BASELINE'
            else:
                # 直近のサンプルを区間代表として積算。共通busなので左右を二重加算しない。
                consumed = max(0.0, current)*dt/3600.0
                self._consumed_ah += consumed
                self._soc = max(0.0, min(100.0, self._soc-100.0*consumed/self.capacity))
                mode = 'COULOMB_COUNTING'
        level = ('HIGH' if voltage > self.high else
                 'CRITICAL' if voltage <= self.critical else
                 'LOW' if voltage <= self.low else 'REFERENCE')
        return self._snapshot(voltage, current if current_valid else None, level, mode)

    def _snapshot(self, voltage, current, level, mode):
        """基準電圧と積算量も渡し、UIから推定方式を確認できるようにする。"""
        return {'voltage_v': voltage, 'current_a': current, 'percent': self._soc,
                'level': level, 'estimate_mode': mode, 'capacity_ah': self.capacity,
                'anchor_voltage_v': self._anchor_voltage, 'consumed_ah': self._consumed_ah}

    def _percent(self, voltage):
        """汎用曲線の隣接点を線形補間し、表示範囲を0〜100%に制限する。"""
        if voltage <= self.curve[0][0]:
            return 0.0
        for (v0, p0), (v1, p1) in zip(self.curve, self.curve[1:]):
            if voltage <= v1:
                return p0+(p1-p0)*(voltage-v0)/(v1-v0)
        return 100.0
