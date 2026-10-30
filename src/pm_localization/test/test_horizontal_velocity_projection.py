"""傾斜時の速度と不確実性が水平座標へ変換されることを検証する。"""
import math
import numpy as np

from pm_localization.horizontal_velocity_projection_node import project_twist


def test_uphill_forward_speed_is_projected():
    values = [1., 0., 0., 0., 0., 0.]
    result, cov = project_twist(values, np.eye(6).reshape(-1), 0., -math.pi/6,
                                np.zeros((2, 2)))
    assert np.allclose(result[:3], [math.sqrt(3)/2, 0., .5])
    assert np.allclose(cov, np.eye(6))


def test_pitch_uncertainty_increases_horizontal_variance():
    values = [1., 0., 0., 0., 0., 0.]
    _, cov = project_twist(values, np.zeros(36), 0., -math.pi/6,
                           np.diag([0., .01]))
    assert np.isclose(cov[0, 0], .0025, rtol=1e-4)
    assert np.allclose(cov, cov.T)


def test_vio_vertical_velocity_contributes_to_horizontal_velocity():
    values = [0., 0., 1., 0., 0., 0.]
    result, _ = project_twist(values, np.zeros(36), 0., -math.pi/6,
                              np.zeros((2, 2)))
    assert np.allclose(result[:3], [-.5, 0., math.sqrt(3)/2])
