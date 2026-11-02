"""曲率から参照速度を作る。Path更新時のみ計算し、高さ・地形costは扱わない。"""
import numpy as np


def build_speed_profile(path, station, target_speed, yaw_rate_limit, yaw_accel_limit, linear_accel):
    """弧長[m]上の速度[m/s]。曲がり角の手前と終点で先行して減速する。

    vehicle interfaceと同じp=4の滑らかな旋回減速を速度上限へ使う。
    yaw加速度はv²*dκ/dsの近似から曲率変化部を低速化する。
    横加速度や路面の摩擦を同定した動力学モデルではなく、最終的な指令制約は
    MPPIとvehicle interfaceが適用する。離散Pathの曲率推定に依存する。
    """
    ds=np.diff(station)
    headings=np.unwrap(np.arctan2(np.diff(path[:,1]),np.diff(path[:,0])))
    curvature=np.zeros(len(path))
    if len(path)>2:
        curvature[1:-1]=np.diff(headings)/(.5*(ds[:-1]+ds[1:]))
    ratio=target_speed*np.abs(curvature)/yaw_rate_limit
    speed=target_speed/np.power(1.+np.power(ratio,4),.25)
    gradient=np.abs(np.gradient(curvature,station))
    speed=np.minimum(speed,np.sqrt(yaw_accel_limit/np.maximum(gradient,1e-12)))
    # 終点は停止。後退passで減速余裕、前進passで再加速の余裕を確保する。
    speed[-1]=0.
    for i in range(len(speed)-2,-1,-1):
        speed[i]=min(speed[i],np.sqrt(speed[i+1]**2+2.*linear_accel*ds[i]))
    for i in range(1,len(speed)):
        speed[i]=min(speed[i],np.sqrt(speed[i-1]**2+2.*linear_accel*ds[i-1]))
    return speed,curvature
