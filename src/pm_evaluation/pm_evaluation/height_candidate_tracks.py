"""オフライン専用の高さ候補対応・短期平均。不存在証拠は推論しない。

同じodom XYセル内で高さ区間とXY外包が近い候補を一対一対応する。
これは物体追跡やpose補正ではなく、対応gateの感度を調べるための診断。
非観測・遮蔽・ground分類変化を負の証拠に変換しない。短期平均は候補ごとに
行い、groundと高点、別セルを混ぜてterrain三指標を再計算しない。
"""
from collections import Counter, deque

import numpy as np


class CandidateTracks:
    """直近N回の対応観測を保持する。窓は秒数でなく有効な対応観測回数。"""

    def __init__(self, window=3, z_gate=.05, xy_gate=.03):
        if window < 1 or z_gate <= 0 or xy_gate < 0:
            raise ValueError('window>=1、z_gate>0、xy_gate>=0が必要です')
        self.window, self.z_gate, self.xy_gate = window, z_gate, xy_gate
        self.cells = {}
        self.next_id = 0
        self.stats = Counter()

    @staticmethod
    def compatible(old, new, z_gate, xy_gate):
        """ground／objectを混ぜず、区間間の最短距離[m]で対応可能性を判定。"""
        if (old['kind'] == 'ground_provisional') != (new['kind'] == 'ground_provisional'):
            return False
        # 広い壁区間の端へ薄い群が触れただけで同じ面の平均に混ぜない。
        if (old['kind'] == 'broad') != (new['kind'] == 'broad'):
            return False
        if old['kind'] == 'broad':
            overlap = max(0., min(old['z_hi_m'], new['z_hi_m'])-max(old['z_lo_m'], new['z_lo_m']))
            union = max(old['z_hi_m'], new['z_hi_m'])-min(old['z_lo_m'], new['z_lo_m'])
            if overlap < .5*union:
                return False
        elif abs(old['z_mean_m']-new['z_mean_m']) > z_gate:
            return False
        gap = max(old['z_lo_m']-new['z_hi_m'], new['z_lo_m']-old['z_hi_m'], 0)
        xgap = max(old['x_lo_m']-new['x_hi_m'], new['x_lo_m']-old['x_hi_m'], 0)
        ygap = max(old['y_lo_m']-new['y_hi_m'], new['y_lo_m']-old['y_hi_m'], 0)
        return gap <= z_gate and np.hypot(xgap, ygap) <= xy_gate

    def update(self, cell, rows, stamp, frame_index):
        """距離の小さい組から一対一に割当。未対応trackは消さず中立に保持。

        平均窓に入れるのは対応した観測だけ。high-hitはそのframeの有効平面から
        判定済みのbool／Noneを入力する。Noneはunknownでありfalseではない。
        """
        tracks = self.cells.setdefault(cell, [])
        old_count = len(tracks)
        pairs = []
        for i, track in enumerate(tracks):
            for j, row in enumerate(rows):
                if self.compatible(track['last'], row, self.z_gate, self.xy_gate):
                    pairs.append((abs(track['last']['z_mean_m']-row['z_mean_m']), i, j))
        used_old, used_new, assignment = set(), set(), {}
        for _, i, j in sorted(pairs):
            if i not in used_old and j not in used_new:
                used_old.add(i); used_new.add(j); assignment[j] = i
        output = []
        for j, row in enumerate(rows):
            if j in assignment:
                track = tracks[assignment[j]]
                self.stats['matched'] += 1
                self.stats['consecutive_matches'] += int(track['frame_index'] == frame_index-1)
                delta = row['z_mean_m']-track['last']['z_mean_m']
            else:
                track = dict(id=self.next_id, history=deque(maxlen=self.window), first_stamp=stamp)
                self.next_id += 1; tracks.append(track)
                self.stats['new'] += 1
                delta = None
            # 全候補CSVの画素・診断列を3回分複製しない。平均／支持に必要な4値のみ。
            track['history'].append({name: row[name] for name in
                                     ('z_mean_m', 'z_lo_m', 'z_hi_m', 'high_hit')})
            track.update(last={name: row[name] for name in
                         ('kind', 'z_mean_m', 'z_lo_m', 'z_hi_m',
                          'x_lo_m', 'x_hi_m', 'y_lo_m', 'y_hi_m')},
                         stamp=stamp, frame_index=frame_index)
            history = track['history']
            # 均等重みの診断平均。点数を重みにするとdense frameを過大評価しうる。
            # broadは平均高さだけで置換せず、直近区間外包も別に保存する。
            hit = [r['high_hit'] for r in history if r['high_hit'] is not None]
            confirmed = len(hit) == self.window and all(hit)
            if row['high_hit'] is not None and bool(row['high_hit']):
                track.setdefault('first_positive_stamp', stamp)
            first_confirmation = confirmed and 'first_confirmed_stamp' not in track
            if first_confirmation:
                track['first_confirmed_stamp'] = stamp
            output.append(dict(track_id=track['id'], matched=j in assignment,
                delta_z_m=delta, mean_z_m=float(np.mean([r['z_mean_m'] for r in history])),
                lo_envelope_m=min(r['z_lo_m'] for r in history),
                hi_envelope_m=max(r['z_hi_m'] for r in history),
                history_size=len(history), positive_support=sum(hit), confirmed=confirmed,
                pending=any(hit) and not confirmed,
                first_stamp=track['first_stamp'], first_confirmation=first_confirmation,
                first_positive_stamp=track.get('first_positive_stamp')))
        self.stats['unmatched_old_neutral'] += old_count-len(used_old)
        return output
