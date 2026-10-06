"""独立frame内の高さ範囲付き候補抽出。時間融合・物体認識は行わない。

高さgapで点を集約するが、候補の内部全体をoccupiedと仮定しない。
平面は任意の診断基準であり、壁・枝の候補に平面fitを要求しない。
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CandidateOptions:
    """距離はm。gap／厚みは障害物のhazard閾値と別の診断parameter。"""

    gap_m: float = 0.05
    thin_span_m: float = 0.04
    ground_tolerance_m: float = 0.03
    max_object_candidates: int = 2
    vehicle_height_m: float = None
    safety_margin_m: float = 0.05

    def __post_init__(self):
        for name in ('gap_m', 'thin_span_m', 'ground_tolerance_m'):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(name + 'は有限の正値が必要です')
        if (isinstance(self.max_object_candidates, bool)
                or not isinstance(self.max_object_candidates, int)
                or self.max_object_candidates < 1):
            raise ValueError('候補枠は1以上の整数が必要です')
        if self.vehicle_height_m is not None and (
                not np.isfinite(self.vehicle_height_m) or self.vehicle_height_m <= 0):
            raise ValueError('車両全高は正値またはnullです')
        if not np.isfinite(self.safety_margin_m) or self.safety_margin_m < 0:
            raise ValueError('安全余裕は有限の非負値が必要です')


def odom_planes(baseline, relative_offset):
    """relative elevationの平面をodom-zへ戻す。傾斜係数は変えない。

    gridはz_relative=z_odom+offsetを保持するので、切片からoffsetを引く。
    offsetを加算すると共通zずれが二重に入り、残差hitの診断を誤る。
    """
    return np.stack((baseline['plane_a'], baseline['plane_b'],
                     baseline['plane_c'] - relative_offset), axis=-1)


def plane_reasons(baseline, parameters):
    """暫定平面gateの棄却理由を重複可能なbitと別layerで返す。

    terrain_featuresの条件を変えず説明する。supportが十分で係数が非有限なら
    determinant gateによる退化。係数成立後のslope／roughness棄却と区別する。
    """
    fresh = (np.isfinite(baseline['relative_elevation'])
             & np.isfinite(baseline['age_seconds'])
             & (baseline['age_seconds'] <= parameters['feature_max_observation_age']))
    support_low = baseline['support_count'] < max(3, parameters['feature_min_neighbors'])
    finite = np.isfinite(odom_planes(baseline, 0)).all(axis=-1)
    reasons = dict(plane_self_unknown=~fresh, plane_support_low=support_low,
                   plane_degenerate=fresh & ~support_low & ~finite,
                   plane_slope_rejected=np.isfinite(baseline['slope_deg'])
                   & (baseline['slope_deg'] >= parameters['hazard_slope_limit_deg']),
                   plane_roughness_rejected=np.isfinite(baseline['roughness'])
                   & (baseline['roughness'] >= parameters['hazard_roughness_limit']))
    bits = np.zeros(fresh.shape, dtype=np.uint8)
    for index, value in enumerate(reasons.values()):
        bits |= value.astype(np.uint8) << index
    return dict(reasons, plane_reason_bits=bits,
                plane_usable=finite & (bits == 0))


def extract_candidates(points, pixels, origin, shape, resolution, planes,
                       plane_valid, obstacle_limit, options):
    """点列をセル・高さ順にsortし、元画素への対応を保って候補を抽出する。

planesは各セル中心基準のa,b,c：z=a*dx+b*dy+c。plane_validは呼出側の
品質gateであり真のgroundの保証ではない。最下群が平面と整合すれば
ground_provisionalとするだけで、天井や岩の上面を排除できたとはしない。
枠外候補もcountと直接hitに寄与し、上限により危険を消さない。
"""
    points = np.asarray(points, dtype=float)
    pixels = np.asarray(pixels)
    h, w = shape
    size = h * w
    if (points.ndim != 2 or points.shape[1] != 3
            or pixels.shape != (len(points), 2)):
        raise ValueError('points=N×3、pixels=N×2が必要です')
    if not np.isfinite(resolution) or resolution <= 0 or min(h, w) < 1:
        raise ValueError('grid寸法・resolutionが不正です')
    if not np.isfinite(obstacle_limit) or obstacle_limit <= 0:
        raise ValueError('obstacle_limitは有限の正値です')
    planes = np.asarray(planes).reshape(size, 3)
    plane_valid = np.asarray(plane_valid, dtype=bool).reshape(size)
    finite = np.isfinite(points).all(axis=1)
    selected = np.flatnonzero(finite)
    ix = np.floor((points[finite, 0] - origin[0]) / resolution).astype(int)
    iy = np.floor((points[finite, 1] - origin[1]) / resolution).astype(int)
    inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    selected, ix, iy = selected[inside], ix[inside], iy[inside]
    slots = iy * w + ix
    order = np.lexsort((points[selected, 2], slots))
    selected, slots = selected[order], slots[order]
    # 原画像全体は保持しない。各frameの診断layerは固定サイズの数値配列。
    layers = {name: np.zeros(size, dtype=np.int32) for name in
              ('point_count', 'candidate_count', 'stored_objects', 'overflow_count',
               'sparse_count', 'broad_count', 'ground_provisional')}
    layers.update({name: np.full(size, np.nan) for name in
                   ('raw_span', 'above_plane_hit', 'body_height_hit')})
    layers.update({name: np.full(size, -1, dtype=np.int32) for name in
                   ('above_plane_witness_index', 'body_height_witness_index')})
    records = []
    starts = np.r_[0, np.flatnonzero(np.diff(slots)) + 1, len(slots)] if len(slots) else []
    for start, end in zip(starts[:-1], starts[1:]):
        slot = int(slots[start])
        ids = selected[start:end]
        xyz = points[ids]
        z = xyz[:, 2]
        layers['point_count'][slot] = len(ids)
        layers['raw_span'][slot] = z[-1] - z[0]
        cx = origin[0] + (slot % w + 0.5) * resolution
        cy = origin[1] + (slot // w + 0.5) * resolution
        a, b, c = planes[slot]
        usable = bool(plane_valid[slot] and np.isfinite(planes[slot]).all())
        residual = z - (a * (xyz[:, 0] - cx) + b * (xyz[:, 1] - cy) + c)
        # 任意の平面が誤groundならこのhit判定も誤る。出力名に基準を明示する。
        if usable:
            hit = residual >= obstacle_limit
            layers['above_plane_hit'][slot] = np.any(hit)
            if np.any(hit):
                layers['above_plane_witness_index'][slot] = ids[np.flatnonzero(hit)[0]]
            if options.vehicle_height_m is not None:
                body_hit = hit & (residual <= options.vehicle_height_m + options.safety_margin_m)
                layers['body_height_hit'][slot] = np.any(body_hit)
                if np.any(body_hit):
                    layers['body_height_witness_index'][slot] = ids[np.flatnonzero(body_hit)[0]]
        bounds = np.r_[0, np.flatnonzero(np.diff(z) > options.gap_m) + 1, len(z)]
        layers['candidate_count'][slot] = len(bounds) - 1
        objects = 0
        for group, (lo, hi) in enumerate(zip(bounds[:-1], bounds[1:])):
            group_ids = ids[lo:hi]
            span = float(z[hi - 1] - z[lo])
            sparse = hi - lo == 1
            # 最小群を無条件groundにしない。平面と整合しても仮説止まり。
            ground = (group == 0 and usable and span <= options.thin_span_m
                      and np.all(np.abs(residual[lo:hi]) <= options.ground_tolerance_m))
            layers['sparse_count'][slot] += int(sparse)
            layers['broad_count'][slot] += int(span > options.thin_span_m)
            if ground:
                layers['ground_provisional'][slot] = 1
            else:
                objects += 1
            stored = ground or objects <= options.max_object_candidates
            if not stored:
                layers['overflow_count'][slot] += 1
                continue
            layers['stored_objects'][slot] += int(not ground)
            lower, upper = group_ids[0], group_ids[-1]
            # 下端／上端は観測点の外包。単一点spread=0を測定誤差0と解釈しない。
            records.append(dict(
                slot=slot, group_index=group, kind='ground_provisional' if ground else (
                    'sparse' if sparse else 'thin' if span <= options.thin_span_m else 'broad'),
                count=hi-lo, z_lo_m=float(z[lo]), z_hi_m=float(z[hi-1]),
                z_mean_m=float(np.mean(z[lo:hi])), z_span_m=span,
                x_lo_m=float(points[group_ids, 0].min()), x_hi_m=float(points[group_ids, 0].max()),
                y_lo_m=float(points[group_ids, 1].min()), y_hi_m=float(points[group_ids, 1].max()),
                min_u=int(pixels[lower, 0]), min_v=int(pixels[lower, 1]),
                max_u=int(pixels[upper, 0]), max_v=int(pixels[upper, 1]),
                min_point_index=int(lower), max_point_index=int(upper),
                plane_usable=usable,
                min_residual_m=float(np.min(residual[lo:hi])) if usable else None,
                max_residual_m=float(np.max(residual[lo:hi])) if usable else None,
                hit_support_frames=1, existence_probability=None,
                free_evidence=False))
    return records, {k: v.reshape(shape) for k, v in layers.items()}
