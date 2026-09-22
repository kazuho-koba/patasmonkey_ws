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
