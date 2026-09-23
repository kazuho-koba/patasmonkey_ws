"""OAK depthをvectorizedにsamplingし、pinhole back-projectする補助関数。

返す座標はROS optical convention（x右、y下、z前方）のm単位である。`odom`への
変換は意図的に分離し、呼び出し側がdepth画像のtimestampに対応するTFを適用する。
"""

import numpy as np


def sampled_points(
    depth_message, fx, fy, cx, cy, stride, min_depth, max_depth,
    return_pixels=False,
):
    """PointCloud2を生成せず、Nx3 optical-frame点列を返す。

    OAKの`16UC1`値はoptical-z方向のmm値である。`frombuffer`はROS payloadを全画像
    copyなしにviewし、sampling後のfloat配列だけを作業用に確保する。`stride`は両pixel
    軸に適用するため、4なら画像の約1/16を評価する。
    """
    if depth_message.encoding not in ("16UC1", "mono16"):
        raise ValueError("expected 16UC1/mono16 depth, got " + depth_message.encoding)
    dtype = ">u2" if depth_message.is_bigendian else "<u2"
    # `step`にはrow paddingが含まれ得るため、まずword数でreshapeしてから各rowを
    # 宣言済みpixel幅へsliceする。
    row_words = depth_message.step // 2
    depth = np.frombuffer(depth_message.data, dtype=dtype).reshape(
        depth_message.height, row_words
    )[:, : depth_message.width]
    # このpixel index列は下のstrided depth viewと同じsampling位相を持ち、pinhole
    # projection用のu/v座標となる。
    rows = np.arange(0, depth_message.height, stride, dtype=np.int32)
    cols = np.arange(0, depth_message.width, stride, dtype=np.int32)
    # range判定の前にmmをmへ変換し、使用可能なdepthだけを残す。0 depthは特別扱いせず
    # min_depth判定で除外される。
    sampled = depth[::stride, ::stride].astype(np.float32) * 0.001
    valid = np.isfinite(sampled) & (sampled >= min_depth) & (sampled <= max_depth)
    if not np.any(valid):
        empty_points = np.empty((0, 3), dtype=np.float32)
        if return_pixels:
            empty_pixels = np.empty(0, dtype=np.int32)
            empty_depth = np.empty(0, dtype=np.float32)
            return empty_points, empty_pixels, empty_pixels.copy(), empty_depth
        return empty_points
    # pinhole back-projection。zは観測したaxial depth、x/yはprincipal pointからの
    # pixel距離に対応する横方向offsetである。
    u, v = np.meshgrid(cols, rows)
    optical_z = sampled[valid]
    optical_x = (u[valid] - cx) * optical_z / fx
    optical_y = (v[valid] - cy) * optical_z / fy
    points = np.column_stack((optical_x, optical_y, optical_z)).astype(
        np.float32, copy=False
    )
    if not return_pixels:
        return points

    # forensic時だけ、各3D sampleを作った元画素とaxial depthも返す。通常経路では
    # この追加配列を確保しないため、Jetsonの通常実行コストを増やさない。
    pixel_u, pixel_v = np.meshgrid(cols, rows)
    return points, pixel_u[valid], pixel_v[valid], optical_z


def quaternion_matrix(quaternion):
    """geometry_msgs Quaternionから正規化済み3x3回転行列を返す。

    ほぼゼロのquaternionはmapping pathでNaNを作らないようidentityとして扱う。有効なTF
    transformは正規化済みであることを期待する。
    """
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
    """1個のTF transformをvectorized NumPy計算で全Nx3点に適用する。"""
    rotation = quaternion_matrix(transform.rotation)
    translation = np.array(
        [transform.translation.x, transform.translation.y, transform.translation.z]
    )
    # 点はrow vectorなので、各rowへの`@ R.T`はcolumn vector表記の`R * point`に
    # 相当し、その後にtarget frameのtranslationを加える。
    return points @ rotation.T + translation
