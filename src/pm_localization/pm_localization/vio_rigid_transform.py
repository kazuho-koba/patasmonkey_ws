"""VIO adapterとオフライン監査で共用する剛体変換。

R_ABはB成分をA成分へ写す回転、p_ABはB原点のA成分位置。
入力poseのorientationはROS HamiltonのR_GI、twistはIMU原点・IMU軸。
OpenVINSのpose共分散は[p_GI, theta_I]の混合座標であることに注意する。
初期配置と取付変換は既知定数として扱い、その校正誤差・初期推定との
相関、およびpose/twist間相関はここでは加えない。
"""
import numpy as np


def skew(v):
    """外積行列。skew(a) @ b = a × b。"""
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def _rotation(value):
    """軸反転・非正規行列を回転として黙認しない。"""
    r = np.asarray(value, dtype=float)
    if (r.shape != (3, 3) or not np.isfinite(r).all()
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-7)
            or not np.isclose(np.linalg.det(r), 1., atol=1e-7)):
        raise ValueError('正規直交・det=+1の回転が必要')
    return r


def _covariance(value):
    """ROSの36要素または6×6の共分散を検査する。負分散は補正せず拒否。"""
    p = np.asarray(value, dtype=float).reshape(6, 6)
    if not np.isfinite(p).all() or not np.allclose(p, p.T, atol=1e-8):
        raise ValueError('共分散は有限・対称であることが必要')
    p = (p + p.T) / 2
    if np.linalg.eigvalsh(p).min() < -1e-8 * max(1., np.linalg.norm(p, 2)):
        raise ValueError('共分散が半正定値ではない')
    return p


def transform_pose(position, rotation_global_imu, covariance,
                   rotation_base_imu, translation_base_imu,
                   rotation_odom_global, translation_odom_global,
                   orientation_error_frame='imu'):
    """IMU poseをbase poseへ変換し、ROS固定軸微小角の共分散を返す。

    入力の位置誤差はglobal[m]、角度誤差は既定でIMU局所軸[rad]。
    OpenVINSではR_GI_true = R_GI Exp(theta_I)となる。既にROS固定軸の
    共分散へ変換済みの場合だけorientation_error_frame='global'を指定する。
    出力順序は(p_OB, R_OB, P_pose, Jacobian)。位置・角度の交差項も保持する。
    """
    r = _rotation(rotation_global_imu)
    c = _rotation(rotation_base_imu)
    a = _rotation(rotation_odom_global)
    p = np.asarray(position, dtype=float).reshape(3)
    t = np.asarray(translation_base_imu, dtype=float).reshape(3)
    offset = np.asarray(translation_odom_global, dtype=float).reshape(3)
    if not all(np.isfinite(v).all() for v in (p, t, offset)):
        raise ValueError('位置・平行移動は有限であることが必要')
    # base原点のIMU成分位置。右からT_IBを掛けて同じ剛体上の原点を移す。
    lever_imu = -c.T @ t
    lever_global = r @ lever_imu
    j = np.zeros((6, 6))
    j[:3, :3] = a
    if orientation_error_frame == 'imu':
        j[:3, 3:] = -a @ r @ skew(lever_imu)
        j[3:, 3:] = a @ r
    elif orientation_error_frame == 'global':
        j[:3, 3:] = -a @ skew(lever_global)
        j[3:, 3:] = a
    else:
        raise ValueError('微小角の座標系はimuまたはglobal')
    cov = j @ _covariance(covariance) @ j.T
    return a @ (p + lever_global) + offset, a @ r @ c.T, (cov + cov.T)/2, j


def transform_twist(twist, covariance, rotation_base_imu, translation_base_imu):
    """IMU原点・IMU軸の[v,w]をbase原点・base軸へ変換する。

    センサ原点速度 v_I^B = v_B^B + w_B × t_BI より、
    v_B^B = C v_I^I + skew(t_BI) C w_I。原点を移してから別の
    軸補正を追加せず、位置ベクトルと角速度を同じbase成分で扱う。
    戻り値は(twist_B, P_twist_B, Jacobian)。
    """
    c = _rotation(rotation_base_imu)
    t = np.asarray(translation_base_imu, dtype=float).reshape(3)
    v = np.asarray(twist, dtype=float).reshape(6)
    if not np.isfinite(v).all() or not np.isfinite(t).all():
        raise ValueError('twist・取付位置は有限であることが必要')
    j = np.zeros((6, 6))
    j[:3, :3] = c
    j[:3, 3:] = skew(t) @ c
    j[3:, 3:] = c
    cov = j @ _covariance(covariance) @ j.T
    return j @ v, (cov + cov.T)/2, j


def yaw_alignment(position_global_imu, rotation_global_imu,
                  rotation_base_imu, translation_base_imu,
                  reference_position, reference_rotation):
    """初回の車体位置・yawをreferenceへ合わせ、固定のT_OGを返す。

    roll/pitchはVIOの重力方向と取付変換から求め、referenceの傾きを
    世界座標へ混入させない。前方軸が鉛直に近い場合はyawを定義できず拒否する。
    """
    r = _rotation(rotation_global_imu)
    c = _rotation(rotation_base_imu)
    reference = _rotation(reference_rotation)
    body = r @ c.T
    if min(np.linalg.norm(body[:2, 0]), np.linalg.norm(reference[:2, 0])) < 1e-4:
        raise ValueError('前方軸が鉛直に近く初期yawが不定')
    angle = np.arctan2(reference[1, 0], reference[0, 0]) - np.arctan2(body[1, 0], body[0, 0])
    cs, sn = np.cos(angle), np.sin(angle)
    a = np.array([[cs, -sn, 0.], [sn, cs, 0.], [0., 0., 1.]])
    center = np.asarray(position_global_imu) - body @ np.asarray(translation_base_imu)
    return a, np.asarray(reference_position) - a @ center
