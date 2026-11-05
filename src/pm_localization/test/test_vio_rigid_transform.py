"""剛体変換を幾何学・数値微分・確率標本で独立に検証する。"""
import numpy as np
import pytest
from pm_localization.vio_rigid_transform import transform_pose, transform_twist, skew
from pm_localization.vio_odom_adapter_node import so3_exp, so3_log

C = np.array([[0., 0., -1.], [0., -1., 0.], [-1., 0., 0.]])


def fixtures():
    r = so3_exp(np.array([.4, -.2, .7]))
    a = so3_exp(np.array([-.1, .3, -.6]))
    t = np.array([.13, -.08, .28])
    rng = np.random.default_rng(917)
    m = rng.normal(size=(6, 6))
    return r, a, t, m @ m.T * 1e-6


def test_pose_matches_homogeneous_transform():
    r, a, t, p = fixtures()
    position = np.array([3., -2., 1.])
    shift = np.array([-1., 4., .7])
    out, rot, _, _ = transform_pose(position, r, p, C, t, a, shift)
    def matrix(rr, tt):
        m = np.eye(4); m[:3, :3] = rr; m[:3, 3] = tt
        return m
    expected = matrix(a, shift) @ matrix(r, position) @ np.linalg.inv(matrix(C, t))
    assert np.allclose(out, expected[:3, 3])
    assert np.allclose(rot, expected[:3, :3])


@pytest.mark.parametrize('error_frame', ['imu', 'global'])
def test_pose_jacobian_by_finite_difference(error_frame):
    r, a, t, cov = fixtures()
    p = np.array([3., 2., -.7])
    out, rot, _, j = transform_pose(p, r, cov, C, t, a, np.zeros(3), error_frame)
    numeric = np.zeros((6, 6))
    for k in range(6):
        delta = np.zeros(6); delta[k] = 1e-7
        # IMU誤差は右乗算、global誤差は左乗算で別々に摂動する。
        rr = r @ so3_exp(delta[3:]) if error_frame == 'imu' else so3_exp(delta[3:]) @ r
        pp, rq, _, _ = transform_pose(p+delta[:3], rr, cov, C, t, a, np.zeros(3), error_frame)
        numeric[:, k] = np.r_[pp-out, so3_log(rq @ rot.T)] / 1e-7
    assert np.allclose(j, numeric, atol=2e-7)


def test_pure_rotation_at_stationary_base_has_zero_linear_twist():
    _, _, t, cov = fixtures()
    omega_base = np.array([.7, -.3, 1.2])
    velocity_imu = C.T @ np.cross(omega_base, t)
    twist = np.r_[velocity_imu, C.T @ omega_base]
    result, _, _ = transform_twist(twist, cov, C, t)
    assert np.allclose(result[:3], 0., atol=1e-14)
    assert np.allclose(result[3:], omega_base)


def test_twist_inverse_recovers_sensor_values():
    _, _, t, cov = fixtures()
    source = np.array([1., .2, -.3, .4, -.7, .1])
    result, pc, _ = transform_twist(source, cov, C, t)
    back, pb, _ = transform_twist(result, pc, C.T, -C.T @ t)
    assert np.allclose(back, source)
    assert np.allclose(pb, cov)


def test_covariance_matches_monte_carlo_including_cross_blocks():
    r, a, t, cov = fixtures()
    rng = np.random.default_rng(442)
    errors = rng.multivariate_normal(np.zeros(6), cov, size=18000)
    out, rot, predicted, _ = transform_pose(np.zeros(3), r, cov, C, t, a, np.zeros(3))
    samples = []
    for error in errors:
        # 共分散の式を使わず、同次変換の定義から位置と姿勢の標本を作る。
        rr = r @ so3_exp(error[3:])
        pp = a @ (error[:3] - rr @ C.T @ t)
        rq = a @ rr @ C.T
        samples.append(np.r_[pp-out, so3_log(rq @ rot.T)])
    observed = np.cov(np.asarray(samples), rowvar=False)
    assert np.linalg.norm(observed-predicted)/np.linalg.norm(predicted) < .035
    assert np.linalg.norm(predicted[:3, 3:]) > 0
    assert np.linalg.eigvalsh(predicted).min() >= -1e-15


def test_orientation_error_frame_conversion_is_equivalent():
    r, a, t, cov = fixtures()
    convert = np.eye(6); convert[3:, 3:] = r
    p1 = transform_pose(np.zeros(3), r, cov, C, t, a, np.zeros(3))[2]
    p2 = transform_pose(np.zeros(3), r, convert@cov@convert.T, C, t,
                        a, np.zeros(3), 'global')[2]
    assert np.allclose(p1, p2)


def test_invalid_rotation_and_covariance_are_rejected():
    with pytest.raises(ValueError):
        transform_twist(np.zeros(6), np.eye(6), -np.eye(3), np.zeros(3))
    bad = np.eye(6); bad[0, 0] = -1
    with pytest.raises(ValueError):
        transform_twist(np.zeros(6), bad, C, np.zeros(3))


def test_twist_covariance_matches_correlated_samples():
    _, _, t, cov = fixtures()
    value = np.array([.6, -.2, .1, .4, -.1, .7])
    result, predicted, _ = transform_twist(value, cov, C, t)
    samples = np.random.default_rng(91).multivariate_normal(value, cov, size=35000)
    # 姿勢変換関数のJacobianを使わず、原点速度の運動学から標本を計算。
    omega = samples[:, 3:] @ C.T
    speed = samples[:, :3] @ C.T - np.cross(omega, t)
    transformed = np.column_stack([speed, omega])
    assert np.linalg.norm(np.cov(transformed, rowvar=False)-predicted)/np.linalg.norm(predicted) < .03
    assert np.allclose(result, np.r_[C @ value[:3] - np.cross(C @ value[3:], t), C @ value[3:]])


def test_zero_lever_preserves_covariance_eigenvalues_under_rotation():
    _, _, _, cov = fixtures()
    _, result, _ = transform_twist(np.zeros(6), cov, C, np.zeros(3))
    assert np.allclose(np.linalg.eigvalsh(result), np.linalg.eigvalsh(cov))
