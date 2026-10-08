"""車体座標の磁気ベクトルを水平化して表示用の磁北方位を求める。"""
import math


def magnetic_bearing(field, roll, pitch, bias=(0, 0, 0), scale=(1, 1, 1),
                     offset_deg=0, declination_deg=0):
    """北0度・時計回りの方位を返す。x前方/y左/z上の右手座標を仮定する。

    fieldとbiasは同一単位（既存Witドライバはraw register値）。絶対強度の
    Tesla換算には依存しない。scaleは軸別補正で、一般のsoft-iron行列の代用ではない。
    roll/pitch [rad]のみで水平化し、磁北に対する方位にoffset/偏角 [deg]を加える。
    この値はGUI専用で、EKFやnavsat_transformへは渡さない。
    """
    if len(field) != 3 or len(bias) != 3 or len(scale) != 3:
        raise ValueError('磁気ベクトル・bias・scaleは3要素必要です')
    if not all(math.isfinite(v) for v in (*field, *bias, *scale, roll, pitch,
                                         offset_deg, declination_deg)):
        raise ValueError('方位入力に非有限値があります')
    if any(v <= 0 for v in scale):
        raise ValueError('scaleは正の値が必要です')
    if abs(pitch) >= math.radians(75):
        raise ValueError('大きなpitchでは方位を表示しません')
    mx, my, mz = [(v-b)*s for v,b,s in zip(field,bias,scale)]
    # Ry(pitch) Rx(roll)で重力に対する傾きを除去する。yawを使うと循環する。
    hx = math.cos(pitch)*mx + math.sin(pitch)*(math.sin(roll)*my+math.cos(roll)*mz)
    hy = math.cos(roll)*my-math.sin(roll)*mz
    if math.hypot(hx,hy) <= 1e-9:
        raise ValueError('水平磁場がゼロで方位を計算できません')
    return (math.degrees(math.atan2(hy,hx))+offset_deg+declination_deg) % 360


def cardinal(bearing):
    """北0度の方位を16方位表記へ変換する。"""
    names = ('N','NNE','NE','ENE','E','ESE','SE','SSE',
             'S','SSW','SW','WSW','W','WNW','NW','NNW')
    return names[int((bearing+11.25)//22.5) % 16]
