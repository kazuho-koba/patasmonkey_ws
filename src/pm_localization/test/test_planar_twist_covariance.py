"""2D仮想観測への分散置換で不定行列を作らないことを検証。"""
import numpy as np
from pm_localization.planar_twist_covariance_node import planar_twist_covariance


def test_preserves_planar_marginal_and_survives_foxy_force_two_d():
    # vxとvzに強い相関がある正常な3D分布。
    source = np.eye(6)*.01
    source[0,2] = source[2,0] = .009
    source[0,1] = source[1,0] = .001
    assert np.linalg.eigvalsh(source).min() > 0
    broken = source.copy()
    for i in (2,3,4):broken[i,i] = 1e-6
    assert np.linalg.eigvalsh(broken).min() < 0
    result = planar_twist_covariance(source)
    idx = np.ix_([0,1,5],[0,1,5])
    assert np.array_equal(result[idx],source[idx])
    for i in (2,3,4):result[i,i] = 1e-6
    assert np.linalg.eigvalsh(result).min() >= 0


def test_correlated_full_rank_covariance_keeps_planar_measurement_information():
    rng = np.random.default_rng(791)
    a = rng.normal(size=(6,6));source = a@a.T
    result = planar_twist_covariance(source)
    assert np.allclose(result[:2,:2],source[:2,:2])
    assert np.linalg.eigvalsh(result).min() >= 0
    assert not result[0,2] and not result[1,3]


def test_projected_wheel_large_unknown_variance_also_needs_marginalization():
    from pm_localization.horizontal_velocity_projection_node import project_twist
    cov = np.diag([.005,1e6,1e6,.01,.01,.1])
    _, projected = project_twist([1.,0.,0.,0.,0.,0.],cov,.1,.2,np.eye(2)*.001)
    broken = projected.copy()
    for i in (2,3,4):broken[i,i]=1e-6
    assert np.linalg.eigvalsh(broken).min()<0
    result=planar_twist_covariance(projected)
    for i in (2,3,4):result[i,i]=1e-6
    assert np.linalg.eigvalsh(result).min()>=0
