"""同じground候補の短期推定と、地形入力の時刻不整合を調べる補助処理。

通常mapperには接続しない。窓は対応候補観測の数で、欠測を0で補わない。
窓未充足時は利用可能な観測を使用し、実際の充填率を別に記録する。
"""
from collections import deque

import numpy as np

from pm_perception.terrain_features import compute_terrain_features


MODES = ('ground_N1', 'mean_N3', 'mean_N5', 'median_N3', 'median_N5')


class GroundHistory:
    """候補IDごとに高さ[m]・撮像stamp[ns]を最大5組保持する。"""

    def __init__(self):
        self.histories = {}

    def update(self, track_id, stamp, height):
        if not np.isfinite(height):
            raise ValueError('ground高さは有限値が必要です')
        history = self.histories.setdefault(track_id, deque(maxlen=5))
        if history and stamp <= history[-1][0]:
            raise ValueError('同trackのstampは単調増加が必要です')
        history.append((int(stamp), float(height)))
        estimates = {}
        for mode in MODES:
            n = int(mode[-1]); selected = list(history)[-n:]
            heights = [row[1] for row in selected]
            value = np.median(heights) if mode.startswith('median') else np.mean(heights)
            estimates[mode] = (float(value), [row[0] for row in selected])
        return estimates


def features(heights, parameters, yaw):
    """同じ現frame ground maskで平面を再fitする。高さだけを方式間で変える。

    age=0は現在のground候補が存在することを表す。履歴の古さが0という意味ではない。
    その履歴stampはseam_diagnosticsへ別入力する。新たな品質gateは追加しない。
    """
    ages = np.where(np.isfinite(heights), 0., np.nan)
    return compute_terrain_features(heights, ages, parameters['resolution'],
        parameters['feature_max_observation_age'], parameters['feature_neighborhood_radius_cells'],
        parameters['feature_min_neighbors'], parameters['hazard_slope_limit_deg'],
        parameters['hazard_roughness_limit'], parameters['hazard_step_limit'],
        heading_yaw=yaw, step_min_side_neighbors=parameters['step_min_side_neighbors'],
        include_diagnostics=True)


def seam_diagnostics(heights, reference, stamps, now):
    """隣接セルの高さ差と履歴差を返す。真の継ぎ目誤差を確定する量ではない。

    stampsは(5,H,W)、未使用枠=-1、観測は右詰め。現在の両セルは同じ画像で
    ground候補を持つが、それ以前に平均した画像集合は異なり得る。
    """
    valid = np.isfinite(heights)
    age = np.where(stamps >= 0, (now-stamps)/1e9, 0.)
    count = (stamps >= 0).sum(axis=0)
    centroid = age.sum(axis=0)/np.maximum(count, 1)
    result = {k: [] for k in ('neighbor_gap_m','gap_increase_vs_N1_m','history_centroid_gap_seconds',
                              'different_history','window_age_seconds','window_size',
                              'gap_increase_different_history_m','gap_increase_same_history_m')}
    result['window_age_seconds'] = age.max(axis=0)[valid]
    result['window_size'] = count[valid]
    for a,b in (((slice(None),slice(None,-1)),(slice(None),slice(1,None))),
                ((slice(None,-1),slice(None)),(slice(1,None),slice(None)))):
        ok = valid[a] & valid[b]
        gap = np.abs(heights[a]-heights[b])
        original = np.abs(reference[a]-reference[b])
        result['neighbor_gap_m'].append(gap[ok])
        result['gap_increase_vs_N1_m'].append(np.maximum(gap-original,0.)[ok])
        result['history_centroid_gap_seconds'].append(np.abs(centroid[a]-centroid[b])[ok])
        different=~np.all(stamps[(slice(None),)+a] == stamps[(slice(None),)+b],axis=0)
        result['different_history'].append(different[ok])
        increase=np.maximum(gap-original,0.)
        result['gap_increase_different_history_m'].append(increase[ok & different])
        result['gap_increase_same_history_m'].append(increase[ok & ~different])
    for key in ('neighbor_gap_m','gap_increase_vs_N1_m','history_centroid_gap_seconds','different_history',
                'gap_increase_different_history_m','gap_increase_same_history_m'):
        result[key] = np.concatenate(result[key])
    return result


class Samples:
    """有限値だけを保存し、平均・p95を厳密に集計する（オフライン用）。"""

    def __init__(self):
        self.parts = []

    def add(self, values):
        array = np.asarray(values).ravel()
        self.parts.append(array[np.isfinite(array)].astype(np.float32))

    def summary(self):
        values = np.concatenate(self.parts) if self.parts else np.empty(0)
        if not len(values):
            return dict(count=0)
        return dict(count=len(values),mean=float(np.mean(values,dtype=np.float64)),
                    median=float(np.median(values)),p95=float(np.percentile(values,95)),
                    max=float(values.max()))
