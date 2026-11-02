"""OccupancyGridのhazardを円形footprintで保守的に拡張する軽量なMPPI入力。

地形融合・意味認識は行わない。変換済みの原点XY/yawはPathと同一frame。
膨張はmap受信時だけ実施し、候補の評価時は配列参照だけにする。
"""
import math
import numpy as np


class TerrainCost:
    def __init__(self,data,width,height,resolution,origin,radius,lethal=100):
        if not 0<width<=2048 or not 0<height<=2048 or width*height!=len(data):
            raise ValueError('hazard mapの寸法・data長が不正')
        if not all(math.isfinite(v) for v in (resolution,radius,*origin)) or resolution<=0 or radius<=0:
            raise ValueError('hazard mapの解像度・原点・footprintが不正')
        raw=np.asarray(data,dtype=np.int16).reshape(height,width)
        if np.any((raw< -1)|(raw>100)):raise ValueError('hazardは-1または0〜100が必要')
        if not 1<=lethal<=100:raise ValueError('lethal閾値は1〜100が必要')
        self.res=resolution;self.origin=origin;self.width=width;self.height=height
        # unknownとmap外は通行可能と推定しない。100以上は候補不採用。
        raw=np.where((raw<0)|(raw>=lethal),100,raw)
        # セル中心のずれ、線分近似の小誤差を吸収する保守的な余裕。
        reach=radius+resolution;cells=int(math.ceil(reach/resolution))
        if cells>100:raise ValueError('hazard解像度に対してfootprintが大きすぎます')
        padded=np.pad(raw,cells,constant_values=100);expanded=np.zeros_like(raw)
        for dy in range(-cells,cells+1):
            for dx in range(-cells,cells+1):
                if math.hypot(dx,dy)*resolution<=reach:
                    np.maximum(expanded,padded[cells+dy:cells+dy+height,cells+dx:cells+dx+width],out=expanded)
        self.grid=expanded

    def sample(self,x,y):
        """Path座標の点群をgridへ変換し、footprint内最大hazardを返す。"""
        x=np.asarray(x);y=np.asarray(y);dx=x-self.origin[0];dy=y-self.origin[1]
        c=math.cos(self.origin[2]);s=math.sin(self.origin[2])
        ix=np.floor((c*dx+s*dy)/self.res).astype(np.int64)
        iy=np.floor((-s*dx+c*dy)/self.res).astype(np.int64)
        inside=(ix>=0)&(ix<self.width)&(iy>=0)&(iy<self.height)
        return np.where(inside,self.grid[np.clip(iy,0,self.height-1),np.clip(ix,0,self.width-1)],100)

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
