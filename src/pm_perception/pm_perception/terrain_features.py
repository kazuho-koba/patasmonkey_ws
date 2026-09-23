"""rolling elevation gridから低copyで求める局所terrain cue。

計算はmapperのdebug/feature rate（通常2 Hz）だけで実行する。固定3x3相当のNumPy
stencilを用い、Jetson runtime pathではPointCloud変換、SciPy、cellごとのPython loopを
用いない。
"""

import numpy as np


CAUSE_NAMES = ("slope", "roughness", "step", "obstacle")


def _window_slices(height, width, dy, dx):
    """targetから`(dx, dy)`だけ離れたsourceのtarget/source sliceを返す。"""
    target_y = slice(max(0, -dy), min(height, height - dy))
    source_y = slice(max(0, dy), min(height, height + dy))
    target_x = slice(max(0, -dx), min(width, width - dx))
    source_x = slice(max(0, dx), min(width, width + dx))
    return (target_y, target_x), (source_y, source_x)


def compute_terrain_features(
    relative_elevation,
    observation_age_seconds,
    resolution,
    max_observation_age,
    neighborhood_radius_cells,
    min_neighbors,
    slope_limit_deg,
    roughness_limit,
    step_limit,
    obstacle_height=None,
    obstacle_limit=0.2,
    heading_yaw=0.0,
    step_min_side_neighbors=1,
    include_diagnostics=False,
):
    """局所平面をfitし、slope・残差roughness・前後support付きstepを求める。

    roughnessは局所平面に対する残差RMSであり、生の高さばらつきではない。stepは平面残差の
    peak-to-peakで、robot headingの前後双方にfresh点が必要である。従って滑らかな平面は
    roughnessにもstepにも反応しない。`heading_yaw`はmap XY軸で表し、supportのgateにだけ
    用いる。残差範囲を符号付き・方向別のstepへ変換するものではない。いずれもfresh観測
    supportを必要とし、obstacle evidenceは独立して扱う。
    """
    values = np.asarray(relative_elevation, dtype=np.float32)
    age = np.asarray(observation_age_seconds, dtype=np.float32)
    # unknown、invalid、古いcellは全ての局所和の前に除外する。観測がないことを平坦な
    # terrainと解釈してはならない。
    fresh = np.isfinite(values) & np.isfinite(age) & (age <= max_observation_age)
    height, width = values.shape
    shape = values.shape
    radius = max(1, int(neighborhood_radius_cells))
    minimum_support = max(3, int(min_neighbors))

    # `z = a*x + b*y + c`用のnormal equation項。固定offsetを使うことで、genericな
    # batched least-squares solveより低負荷かつ予測可能にする。
    count = np.zeros(shape, dtype=np.uint16)
    sx = np.zeros(shape, dtype=np.float32)
    sy = np.zeros(shape, dtype=np.float32)
    sxx = np.zeros(shape, dtype=np.float32)
    sxy = np.zeros(shape, dtype=np.float32)
    syy = np.zeros(shape, dtype=np.float32)
    sz = np.zeros(shape, dtype=np.float32)
    sxz = np.zeros(shape, dtype=np.float32)
    syz = np.zeros(shape, dtype=np.float32)
    offsets = []
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            # 各source stencilを全target cellへshiftする。dx/dyはm単位で既知のgrid
            # offsetなので、これらの和を蓄積すればcellごとのloopなしにtargetごとの
            # normal equationを作れる。
            target, source = _window_slices(height, width, dy, dx)
            valid = fresh[source]
            z = values[source]
            x_m = dx * resolution
            y_m = dy * resolution
            count[target] += valid
            sx[target] += valid * x_m
            sy[target] += valid * y_m
            sxx[target] += valid * x_m * x_m
            sxy[target] += valid * x_m * y_m
            syy[target] += valid * y_m * y_m
            sz[target] += np.where(valid, z, 0.0)
            sxz[target] += np.where(valid, z * x_m, 0.0)
            syz[target] += np.where(valid, z * y_m, 0.0)
            offsets.append((dx, dy, target, source))

    # 対称3x3 normal equationに対するCramer's rule。determinant gateにより、高価な
    # cellごとのsolverを使わずcollinearまたはsparseなsupportを除外する。
    determinant = (
        sxx * (syy * count - sy * sy)
        - sxy * (sxy * count - sy * sx)
        + sx * (sxy * sy - syy * sx)
    )
    plane_valid = fresh & (count >= minimum_support) & (np.abs(determinant) > 1e-9)
    a = np.zeros(shape, dtype=np.float32)
    b = np.zeros(shape, dtype=np.float32)
    c = np.zeros(shape, dtype=np.float32)
    a_numerator = (
        sxz * (syy * count - sy * sy)
        - sxy * (syz * count - sy * sz)
        + sx * (syz * sy - syy * sz)
    )
    b_numerator = (
        sxx * (syz * count - sy * sz)
        - sxz * (sxy * count - sy * sx)
        + sx * (sxy * sz - syz * sx)
    )
    c_numerator = (
        sxx * (syy * sz - syz * sy)
        - sxy * (sxy * sz - syz * sx)
        + sxz * (sxy * sy - syy * sx)
    )
    a[plane_valid] = a_numerator[plane_valid] / determinant[plane_valid]
    b[plane_valid] = b_numerator[plane_valid] / determinant[plane_valid]
    c[plane_valid] = c_numerator[plane_valid] / determinant[plane_valid]
    if include_diagnostics:
        # 係数配列はoffline診断時だけ返す。通常運転では追加のgrid配列を確保しない。
        plane_a = np.full(shape, np.nan, dtype=np.float32)
        plane_b = np.full(shape, np.nan, dtype=np.float32)
        plane_c = np.full(shape, np.nan, dtype=np.float32)
        plane_a[plane_valid] = a[plane_valid]
        plane_b[plane_valid] = b[plane_valid]
        plane_c[plane_valid] = c[plane_valid]

    # fit後に同じstencilを再走査する。3x3xHxW tensorを実体化せず、残差RMSと残差極値の
    # 両方を求める。
    residual_sq_sum = np.zeros(shape, dtype=np.float32)
    residual_min = np.full(shape, np.inf, dtype=np.float32)
    residual_max = np.full(shape, -np.inf, dtype=np.float32)
    forward_count = np.zeros(shape, dtype=np.uint8)
    rear_count = np.zeros(shape, dtype=np.uint8)
    heading_x = float(np.cos(heading_yaw))
    heading_y = float(np.sin(heading_yaw))
    for dx, dy, target, source in offsets:
        valid = fresh[source]
        z = values[source]
        residual = z - (a[target] * (dx * resolution)
                        + b[target] * (dy * resolution) + c[target])
        residual_sq_sum[target] += np.where(valid, residual * residual, 0.0)
        residual_min[target] = np.minimum(
            residual_min[target], np.where(valid, residual, np.inf)
        )
        residual_max[target] = np.maximum(
            residual_max[target], np.where(valid, residual, -np.inf)
        )
        # offsetとheadingのdot productで前後neighborを識別する。0.5 cellの閾値は、
        # ほぼ側方のsampleをstep supportから除外する。
        projection = dx * heading_x + dy * heading_y
        if projection >= 0.5:
            forward_count[target] += valid
        elif projection <= -0.5:
            rear_count[target] += valid

    slope = np.full(shape, np.nan, dtype=np.float32)
    roughness = np.full(shape, np.nan, dtype=np.float32)
    step_height = np.full(shape, np.nan, dtype=np.float32)
    slope[plane_valid] = np.degrees(np.arctan(np.hypot(
        a[plane_valid], b[plane_valid]
    )))
    roughness[plane_valid] = np.sqrt(
        residual_sq_sum[plane_valid] / np.maximum(count[plane_valid], 1)
    )
    step_valid = (
        plane_valid
        & (forward_count >= max(1, int(step_min_side_neighbors)))
        & (rear_count >= max(1, int(step_min_side_neighbors)))
    )
    # 出力する量は近傍全体の残差範囲のままである。前後supportは片側だけのsparse観測を
    # 抑制するためだけに使う。
    step_height[step_valid] = (
        residual_max[step_valid] - residual_min[step_valid]
    )
    if include_diagnostics:
        residual_min_out = np.full(shape, np.nan, dtype=np.float32)
        residual_max_out = np.full(shape, np.nan, dtype=np.float32)
        residual_min_out[plane_valid] = residual_min[plane_valid]
        residual_max_out[plane_valid] = residual_max[plane_valid]

    # 各cueは独立に使用できる。obstacleは有効な局所平面を必要としないが、
    # roughness/slope/stepは必要とする。
    scores = np.full((4,) + shape, -np.inf, dtype=np.float32)
    for index, (cue, limit) in enumerate((
        (slope, slope_limit_deg),
        (roughness, roughness_limit),
        (step_height, step_limit),
    )):
        valid = np.isfinite(cue)
        scores[index, valid] = cue[valid] / max(float(limit), 1e-6)
    if obstacle_height is not None:
        obstacle = np.asarray(obstacle_height, dtype=np.float32)
        valid = np.isfinite(obstacle)
        scores[3, valid] = obstacle[valid] / max(float(obstacle_limit), 1e-6)
    # `-inf`は欠測cueを表すためmaxから除外される。4 cueすべてが使えないときだけcellを
    # unknownとし、安全（0）とは扱わない。
    max_score = np.max(scores, axis=0)
    hazard_valid = np.isfinite(max_score)
    hazard = np.full(shape, np.nan, dtype=np.float32)
    hazard[hazard_valid] = np.clip(max_score[hazard_valid], 0.0, 1.0)
    max_cause = np.zeros(shape, dtype=np.uint8)
    # np.argmaxは決定的で、同率ならslope、roughness、step、obstacleの順で最初のcueを選ぶ。
    # これはRViz/解析用labelにすぎず、同じcellで別cueもlimit超過していることがある。
    max_cause[hazard_valid] = (
        np.argmax(scores[:, hazard_valid], axis=0).astype(np.uint8) + 1
    )
    result = {
        "slope_deg": slope,
        "roughness": roughness,
        "step_height": step_height,
        "hazard": hazard,
        "support_count": count,
        "step_support_forward": forward_count,
        "step_support_rear": rear_count,
        "max_cause": max_cause,
    }
    if include_diagnostics:
        result.update({
            "plane_a": plane_a,
            "plane_b": plane_b,
            "plane_c": plane_c,
            "residual_rms": roughness,
            "residual_min": residual_min_out,
            "residual_max": residual_max_out,
        })
    return result
