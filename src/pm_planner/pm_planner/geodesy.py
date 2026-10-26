"""WGS84から、明示されたanchorのmetric frameへの変換。

ECEF→ENU→平面frameの順。anchorのeast_yaw_radは「そのframeで東を向くyaw」で、
車体の初期yawと同じとは限らない。最初のwaypointを暗黙の原点にしない。
"""
from dataclasses import dataclass
import math


def ecef(latitude, longitude, altitude):
    """WGS84楕円体の緯度経度[degree]・楕円体高[m]からECEF[m]を返す。"""
    if not all(math.isfinite(v) for v in (latitude, longitude, altitude)):
        raise ValueError('緯度経度・高度は有限値が必要です')
    if abs(latitude) > 90 or abs(longitude) > 180:
        raise ValueError('WGS84緯度経度の範囲外です')
    lat, lon = math.radians(latitude), math.radians(longitude)
    a, f = 6378137., 1./298.257223563
    e2 = f*(2-f); n = a/math.sqrt(1-e2*math.sin(lat)**2)
    return ((n+altitude)*math.cos(lat)*math.cos(lon),
            (n+altitude)*math.cos(lat)*math.sin(lon),
            (n*(1-e2)+altitude)*math.sin(lat))


@dataclass(frozen=True)
class Anchor:
    """地理anchorと対応するframe上の点。位置はm、角度はrad。"""
    latitude: float
    longitude: float
    altitude: float
    frame_id: str
    x: float = 0.
    y: float = 0.
    z: float = 0.
    east_yaw_rad: float = 0.
    alignment_verified: bool = False

    def __post_init__(self):
        ecef(self.latitude, self.longitude, self.altitude)
        if not self.frame_id or not all(math.isfinite(v) for v in (self.x,self.y,self.z,self.east_yaw_rad)):
            raise ValueError('anchorのframeと有限の座標・角度が必要です')

    def project(self, latitude, longitude):
        """高度のないmissionはanchorと同じ楕円体高で水平投影する。

        地形高度や車体姿勢を推定する処理ではない。道路面がこの高さという保証もない。
        """
        p, origin = ecef(latitude,longitude,self.altitude), ecef(self.latitude,self.longitude,self.altitude)
        dx,dy,dz = (p[i]-origin[i] for i in range(3))
        lat,lon = math.radians(self.latitude),math.radians(self.longitude)
        east = -math.sin(lon)*dx+math.cos(lon)*dy
        north = -math.sin(lat)*math.cos(lon)*dx-math.sin(lat)*math.sin(lon)*dy+math.cos(lat)*dz
        c,s = math.cos(self.east_yaw_rad),math.sin(self.east_yaw_rad)
        return self.x+c*east-s*north,self.y+s*east+c*north


def planar_transform(transform, x, y, z, max_tilt):
    """TFの水平成分を使う前にroll/pitchが小さいことを検査する。

    navsat出力frameから計画frameへの2D変換。3D TFを勝手に変更しない。
    大きなtiltではheight未指定missionのXYが一意に扱えないため拒否する。
    """
    q = transform.rotation
    values = (q.x,q.y,q.z,q.w,transform.translation.x,transform.translation.y,transform.translation.z)
    if not all(math.isfinite(v) for v in values):raise ValueError('TFが非有限です')
    norm=math.sqrt(sum(v*v for v in values[:4]))
    if abs(norm-1.)>.01:raise ValueError('TF quaternionが正規化されていません')
    roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
    pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
    if max(abs(roll),abs(pitch))>max_tilt:raise ValueError('地理変換TFが平面仮定のtilt上限を超えています')
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    c,s=math.cos(yaw),math.sin(yaw)
    return transform.translation.x+c*x-s*y,transform.translation.y+s*x+c*y
