"""Low-copy local terrain cues derived from the rolling elevation grid.

The calculation runs only at the mapper's debug/feature rate (normally 2 Hz).
It uses fixed 3x3-style NumPy stencils: no PointCloud conversion, SciPy, or
per-cell Python loops are used in the Jetson runtime path.
"""

import numpy as np


CAUSE_NAMES = ("slope", "roughness", "step", "obstacle")


def _window_slices(height, width, dy, dx):
    """Return target/source slices where source is `(dx, dy)` from target."""
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
):
    """Fit local planes and derive slope, residual roughness and forward step.

    Roughness is the RMS residual to the locally fitted plane, not raw height
    spread. Step is the peak-to-peak plane residual, while fresh points must
    exist on both forward and rear sides of the robot heading. A smooth plane
    therefore produces neither a roughness nor a step response. ``heading_yaw``
    is expressed in the map XY axes and gates support only; it does not make
    the residual range a signed, direction-specific step. Both require fresh
    observation support; obstacle evidence remains independent.
    """
    values = np.asarray(relative_elevation, dtype=np.float32)
    age = np.asarray(observation_age_seconds, dtype=np.float32)
    fresh = np.isfinite(values) & np.isfinite(age) & (age <= max_observation_age)
    height, width = values.shape
    shape = values.shape
    radius = max(1, int(neighborhood_radius_cells))
    minimum_support = max(3, int(min_neighbors))

    # Normal-equation terms for z = a*x + b*y + c. Fixed offsets make this
    # cheaper and more predictable than a batched generic least-squares solve.
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

    # Cramer's rule for symmetric 3x3 normal equations. The determinant gate
    # rejects collinear/sparse support without an expensive per-cell solver.
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
    step_height[step_valid] = (
        residual_max[step_valid] - residual_min[step_valid]
    )

    # Each cue remains independently usable. An obstacle does not need a
    # usable local plane, while roughness/slope/step do require one.
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
    max_score = np.max(scores, axis=0)
    hazard_valid = np.isfinite(max_score)
    hazard = np.full(shape, np.nan, dtype=np.float32)
    hazard[hazard_valid] = np.clip(max_score[hazard_valid], 0.0, 1.0)
    max_cause = np.zeros(shape, dtype=np.uint8)
    # np.argmax is deterministic: ties select the first cue in the order
    # slope, roughness, step, obstacle.  This is an RViz/analysis label only;
    # another cue may also be over its limit in the same cell.
    max_cause[hazard_valid] = (
        np.argmax(scores[:, hazard_valid], axis=0).astype(np.uint8) + 1
    )
    return {
        "slope_deg": slope,
        "roughness": roughness,
        "step_height": step_height,
        "hazard": hazard,
        "support_count": count,
        "step_support_forward": forward_count,
        "step_support_rear": rear_count,
        "max_cause": max_cause,
    }
