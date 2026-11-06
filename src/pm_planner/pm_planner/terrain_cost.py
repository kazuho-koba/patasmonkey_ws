"""OccupancyGridのhazardを円形footprintで保守的に拡張する軽量なMPPI入力。

地形融合・意味認識は行わない。変換済みの原点XY/yawはPathと同一frame。
膨張はmap受信時だけ実施し、候補の評価時は配列参照だけにする。
"""
import math
import numpy as np
import cv2


class TerrainCost:
    def __init__(self,data,width,height,resolution,origin,radius,lethal=100,margin_m=None):
        if not 0<width<=2048 or not 0<height<=2048 or width*height!=len(data):
            raise ValueError('hazard mapの寸法・data長が不正')
        if not all(math.isfinite(v) for v in (resolution,radius,*origin)) or resolution<=0 or radius<=0:
            raise ValueError('hazard mapの解像度・原点・footprintが不正')
        # 旧Python呼出しは未指定時のみ1セル分を維持。通常nodeはm単位の設定を明示する。
        if margin_m is None:margin_m=resolution
        if not math.isfinite(margin_m) or margin_m<0:raise ValueError('追加余裕は非負の有限値[m]が必要')
        self.margin_m=float(margin_m);self.inflation_radius=radius+self.margin_m
        raw=np.asarray(data,dtype=np.int16).reshape(height,width)
        if np.any((raw< -1)|(raw>100)):raise ValueError('hazardは-1または0〜100が必要')
        if not 1<=lethal<=100:raise ValueError('lethal閾値は1〜100が必要')
        self.res=resolution;self.origin=origin;self.width=width;self.height=height
        # unknownとmap外は通行可能と推定しない。100以上は候補不採用。
        raw=np.where((raw<0)|(raw>=lethal),100,raw)
        # 追加余裕は解像度に連動させず、障害物・unknownの両方へ同じm幅で適用する。
        reach=self.inflation_radius;cells=int(math.ceil(reach/resolution))
        if cells>100:raise ValueError('hazard解像度に対してfootprintが大きすぎます')
        padded=np.pad(raw,cells,constant_values=100);expanded=np.zeros_like(raw)
        for dy in range(-cells,cells+1):
            for dx in range(-cells,cells+1):
                # 0.3 m / 0.1 m等の境界を丸め誤差で1セル縮めない。
                if math.hypot(dx,dy)*resolution<=reach+resolution*1e-9:
                    np.maximum(expanded,padded[cells+dy:cells+dy+height,cells+dx:cells+dx+width],out=expanded)
        self.grid=expanded
        # map更新時に一度だけ距離変換する。膨張済み禁止領域（unknown・map外含む）
        # からの余裕[m]を保持し、候補数×予測stepで距離探索を繰り返さない。
        free=np.pad(expanded<100,1,constant_values=False)
        self.clearance_grid=np.maximum(0.,cv2.distanceTransform(free.astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)[1:-1,1:-1]*resolution-resolution)

    def sample(self,x,y):
        """Path座標の点群をgridへ変換し、footprint内最大hazardを返す。"""
        x=np.asarray(x);y=np.asarray(y);dx=x-self.origin[0];dy=y-self.origin[1]
        c=math.cos(self.origin[2]);s=math.sin(self.origin[2])
        ix=np.floor((c*dx+s*dy)/self.res).astype(np.int64)
        iy=np.floor((-s*dx+c*dy)/self.res).astype(np.int64)
        inside=(ix>=0)&(ix<self.width)&(iy>=0)&(iy<self.height)
        return np.where(inside,self.grid[np.clip(iy,0,self.height-1),np.clip(ix,0,self.width-1)],100)

    def clearance_risk(self,x,y,preferred_m):
        """必須膨張の外側で追加確保したい距離に対する不足率[0,1]。

        0は希望余裕以上、1は禁止領域の境界。map外は1とする。
        セル中心間距離から1セル分を差し引き、離散化を保守的に扱う。
        """
        if preferred_m<=0:return np.zeros_like(np.asarray(x),dtype=float)
        dx=np.asarray(x)-self.origin[0];dy=np.asarray(y)-self.origin[1]
        c=math.cos(self.origin[2]);s=math.sin(self.origin[2])
        ix=np.floor((c*dx+s*dy)/self.res).astype(np.int64)
        iy=np.floor((-s*dx+c*dy)/self.res).astype(np.int64)
        inside=(ix>=0)&(ix<self.width)&(iy>=0)&(iy<self.height)
        distance=self.clearance_grid[np.clip(iy,0,self.height-1),np.clip(ix,0,self.width-1)]
        return np.where(inside,np.clip(1.-distance/preferred_m,0.,1.),1.)

    def segment(self,x0,y0,x1,y1):
        """制御step間も最大半セル間隔で検査し、障害物セルの飛び越しを防ぐ。"""
        distance=float(np.max(np.hypot(np.asarray(x1)-x0,np.asarray(y1)-y0)))
        pieces=max(1,int(math.ceil(distance/(self.res*.5))))
        if pieces>100:raise ValueError('hazard検査のstep間隔が大きすぎます')
        cost=np.zeros_like(np.asarray(x1),dtype=float)
        for t in np.linspace(0.,1.,pieces+1):
            cost=np.maximum(cost,self.sample(np.asarray(x0)+(np.asarray(x1)-x0)*t,
                                             np.asarray(y0)+(np.asarray(y1)-y0)*t))
        return cost
