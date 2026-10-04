from types import SimpleNamespace

import numpy as np

from pm_perception.depth_projection import sampled_points


def test_uint16_millimetres_back_project_to_metres():
    depth = np.array([[1000, 0], [2000, 4001]], dtype=np.uint16)
    message = SimpleNamespace(
        encoding="16UC1", is_bigendian=False, step=4, width=2, height=2,
        data=depth.tobytes()
    )
    points = sampled_points(message, 1.0, 1.0, 0.0, 0.0, 1, 0.4, 4.0)
    assert np.allclose(points, [[0.0, 0.0, 1.0], [0.0, 2.0, 2.0]])


def test_forensic_sampling_returns_source_pixel_and_axial_depth():
    depth = np.array([[1000, 0], [2000, 4001]], dtype=np.uint16)
    message = SimpleNamespace(
        encoding="16UC1", is_bigendian=False, step=4, width=2, height=2,
        data=depth.tobytes()
    )
    points, pixel_u, pixel_v, axial_depth = sampled_points(
        message, 1.0, 1.0, 0.0, 0.0, 1, 0.4, 4.0, return_pixels=True
    )
    assert np.allclose(points[:, 2], axial_depth)
    assert np.array_equal(pixel_u, [0, 0])
    assert np.array_equal(pixel_v, [0, 1])
    assert np.allclose(axial_depth, [1.0, 2.0])


def test_stride_four_uses_first_valid_pixel_per_block():
    # 左上が有効なら保持。別区画は0・範囲外を飛ばし、最後の有効画素も見つける。
    depth = np.zeros((4, 12), dtype=np.uint16)
    depth[0, 0], depth[0, 1] = 1000, 2000
    depth[0, 4], depth[0, 5], depth[1, 4] = 100, 5000, 2000
    depth[3, 11] = 3000
    message = SimpleNamespace(encoding="16UC1", is_bigendian=False,
                              step=24, width=12, height=4, data=depth.tobytes())
    points, u, v, z = sampled_points(message, 1, 1, 0, 0, 4, 0.4, 4,
                                    return_pixels=True)
    assert np.array_equal(u, [0, 4, 11])
    assert np.array_equal(v, [0, 1, 3])
    assert np.allclose(z, [1, 2, 3])
    assert np.allclose(points[:, 0], u*z)
    assert np.allclose(points[:, 1], v*z)


def test_partial_blocks_padding_big_endian_and_invalid_blocks():
    depth = np.zeros((5, 8), dtype=">u2")
    depth[:, 5:] = 1000  # row paddingを有効pixelと誤認してはいけない
    depth[4, 4] = 4000  # 右下の1×1区画
    message = SimpleNamespace(encoding="mono16", is_bigendian=True,
                              step=16, width=5, height=5, data=depth.tobytes())
    points, u, v, z = sampled_points(message, 1, 1, 0, 0, 4, 0.4, 4,
                                    return_pixels=True)
    assert points.shape == (1, 3)
    assert np.array_equal(u, [4]) and np.array_equal(v, [4])
    assert np.allclose(z, [4])


def test_zero_never_counts_as_valid_even_with_zero_minimum():
    depth = np.zeros((4, 4), dtype=np.uint16)
    message = SimpleNamespace(encoding="16UC1", is_bigendian=False,
                              step=8, width=4, height=4, data=depth.tobytes())
    assert sampled_points(message, 1, 1, 0, 0, 4, 0, 4).shape == (0, 3)
