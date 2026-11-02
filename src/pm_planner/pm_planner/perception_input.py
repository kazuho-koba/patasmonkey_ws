"""障害物と他の地形の評価を混ぜず選択する、ROS非依存の入力検証。

最大要因ラベルから障害物costを復元しない。fullで必要な片側が欠測なら
unknownを維持する。ただし片側だけで致命的な危険が既知なら危険を優先する。
"""
import numpy as np

MODES=('off','obstacle_only','full_terrain')


def select_costs(obstacle,terrain,size,mode,lethal=100):
    """各channelの-1/0〜100を検証し、選択modeの独立配列を返す。"""
    if mode not in ('obstacle_only','full_terrain'):raise ValueError('認識を使うmodeが必要です')
    if not 0<size<=2048*2048 or len(obstacle)!=size:raise ValueError('obstacle_costの長さが不正')
    if len(terrain) not in (0,size):raise ValueError('terrain_costの長さが不正')
    if not 1<=lethal<=100:raise ValueError('危険閾値は1〜100が必要です')
    a=np.asarray(obstacle,dtype=np.int16)
    b=np.full(size,-1,dtype=np.int16) if len(terrain)==0 else np.asarray(terrain,dtype=np.int16)
    if np.any((a< -1)|(a>100)) or np.any((b< -1)|(b>100)):raise ValueError('costは-1または0〜100です')
    if mode=='obstacle_only':return a.copy()
    known=(a>=0)&(b>=0)
    result=np.where(known,np.maximum(a,b),-1)
    result[(a>=lethal)|(b>=lethal)]=100
    return result
