"""Vectorized depth-image sampling and pinhole back-projection."""

import numpy as np


def sampled_points(depth_message, fx, fy, cx, cy, stride, min_depth, max_depth):
    """Return Nx3 optical-frame points without constructing PointCloud2."""
    if depth_message.encoding not in ("16UC1", "mono16"):
        raise ValueError("expected 16UC1/mono16 depth, got " + depth_message.encoding)
    dtype = ">u2" if depth_message.is_bigendian else "<u2"
    row_words = depth_message.step // 2
    depth = np.frombuffer(depth_message.data, dtype=dtype).reshape(
        depth_message.height, row_words
    )[:, : depth_message.width]
    rows = np.arange(0, depth_message.height, stride, dtype=np.int32)
    cols = np.arange(0, depth_message.width, stride, dtype=np.int32)
    sampled = depth[::stride, ::stride].astype(np.float32) * 0.001
    valid = np.isfinite(sampled) & (sampled >= min_depth) & (sampled <= max_depth)
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float32)
    u, v = np.meshgrid(cols, rows)
    optical_z = sampled[valid]
    optical_x = (u[valid] - cx) * optical_z / fx
    optical_y = (v[valid] - cy) * optical_z / fy
    return np.column_stack((optical_x, optical_y, optical_z)).astype(
        np.float32, copy=False
    )


def quaternion_matrix(quaternion):
    """Return a 3x3 rotation matrix for geometry_msgs Quaternion."""
    q = np.array(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w],
        dtype=np.float64,
    )
    norm = np.dot(q, q)
    if norm < np.finfo(float).eps:
        return np.eye(3)
    q *= np.sqrt(2.0 / norm)
    outer = np.outer(q, q)
    return np.array(
        [
            [
                1.0 - outer[1, 1] - outer[2, 2],
                outer[0, 1] - outer[2, 3],
                outer[0, 2] + outer[1, 3],
            ],
            [
                outer[0, 1] + outer[2, 3],
                1.0 - outer[0, 0] - outer[2, 2],
                outer[1, 2] - outer[0, 3],
            ],
            [
                outer[0, 2] - outer[1, 3],
                outer[1, 2] + outer[0, 3],
                1.0 - outer[0, 0] - outer[1, 1],
            ],
        ],
        dtype=np.float64,
    )


def transform_points(points, transform):
    rotation = quaternion_matrix(transform.rotation)
    translation = np.array(
        [transform.translation.x, transform.translation.y, transform.translation.z]
    )
    return points @ rotation.T + translation
