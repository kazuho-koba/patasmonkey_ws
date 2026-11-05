"""画像内評価を空間保持する有限grid。未知は安全値へ変換しない。

解除はN観測窓の有効高さ平均を使う。欠測のみの窓では記録を解除しunknownへ戻す。
これは物体不存在・ノイズの証明ではない。通常gridとは別実装。
"""
from collections import deque
import numpy as np
from pm_perception.terrain_features import compute_terrain_features


class IndependentObstacles:
    def __init__(self, resolution=.1, size=8., height=.08, window=3,
                 min_points=3,
                 count_ratio=.75, ground_tolerance=.03):
        if not np.isfinite([resolution,size,height,window,min_points,count_ratio,ground_tolerance]).all():
            raise ValueError('parameterは有限値が必要')
        if not 0<count_ratio<=1 or ground_tolerance<=0:raise ValueError('解除品質条件が不正')
        if resolution<=0 or size<=0 or height<=0 or window<1 or min_points<1:
            raise ValueError('grid／閾値／窓／点数／budgetは正値が必要')
        self.res=resolution;self.width=int(round(size/resolution));self.height=self.width
        self.limit=height;self.window=window;self.min_points=min_points
        self.witness_spacing=.03
        self.count_ratio=count_ratio;self.ground_tolerance=ground_tolerance
        self.cells={};self.origin=np.zeros(2);self.last_stamp=-1
        self.counters=dict(hits=0,cleared=0,overflow=0)

    def update(self, points, camera, center, stamp, yaw=0., slope=20., rough=.03, step=.07):
        """点はodom座標[m]。同じ撮像stampのcamera位置とbase中心を入力する。"""
        if stamp<=self.last_stamp:raise ValueError('画像stamp逆行。nodeを再起動してください')
        self.last_stamp=stamp
        points=np.asarray(points,dtype=float);camera=np.asarray(camera,dtype=float)
        if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(camera).all():
            raise ValueError('有限のcameraとNx3点が必要')
        points=points[np.isfinite(points).all(axis=1)]
        origin=np.floor(np.asarray(center[:2])/self.res).astype(int)-self.width//2
        self.origin=origin*self.res
        # rolling領域外は破棄する。時間経過だけでは危険を解除しない。
        self.cells={k:v for k,v in self.cells.items() if
                    origin[0]<=k[0]<origin[0]+self.width and origin[1]<=k[1]<origin[1]+self.height}
        ids=np.floor(points[:,:2]/self.res).astype(int)-origin
        inside=(ids[:,0]>=0)&(ids[:,0]<self.width)&(ids[:,1]>=0)&(ids[:,1]<self.height)
        q=points[inside];ids=ids[inside];slots=ids[:,1]*self.width+ids[:,0]
        total=self.width*self.height
        low=np.full(total,np.inf);high=np.full(total,-np.inf)
        np.minimum.at(low,slots,q[:,2]);np.maximum.at(high,slots,q[:,2])
        count=np.bincount(slots,minlength=total);low[count==0]=np.nan
        shape=(self.height,self.width);z=low.reshape(shape)
        f=compute_terrain_features(z,np.where(np.isfinite(z),0.,np.nan),self.res,3.,1,5,
                                  slope,rough,step,heading_yaw=yaw,step_min_side_neighbors=2,
                                  include_diagnostics=True)
        # 点ごとのXYで平面高さを評価。急斜面だから高点情報を捨てるgateは加えない。
        a=f['plane_a'].ravel();b=f['plane_b'].ravel();c=f['plane_c'].ravel()
        xy=self.origin+(ids+.5)*self.res
        residual=q[:,2]-(a[slots]*(q[:,0]-xy[:,0])+b[slots]*(q[:,1]-xy[:,1])+c[slots])
        maximum=np.full(total,-np.inf);np.maximum.at(maximum,slots,residual)
        # 平面不成立時にもraw幅による危険を保持する。raw幅だけでfreeにはしない。
        danger=(maximum>=self.limit)|((high-low)>=self.limit)
        for slot in np.flatnonzero(count):
            yy,xx=divmod(int(slot),self.width);key=(int(origin[0]+xx),int(origin[1]+yy))
            state=self.cells.setdefault(key,dict(obstacle=-1,terrain=-1,active=False,
                confirmed=False,hits=0,witnesses=[],overflow=False,lows=deque(maxlen=self.window)))
            cues=np.array([f[n][yy,xx] for n in ('slope_deg','roughness','step_height')])
            known=np.isfinite(cues);scores=cues/np.array([slope,rough,step])
            if np.any(scores[known]>=1):state['terrain']=100
            elif known.all():state['terrain']=int(np.rint(np.clip(scores.max(),0,1)*100))
            if danger[slot]:
                self.counters['hits']+=1;state['active']=True;state['obstacle']=100
                state['lows'].clear();state['lows'].append(max(self.limit,float(maximum[slot])) if np.isfinite(maximum[slot]) else float(high[slot]-low[slot]));state['hits']+=1
                # 同セルでの支持回数は診断値。旧試験の群track確認とは区別する。
                state['confirmed']=state['hits']>=self.window
                state['ground']=float(low[slot]);state['points']=int(count[slot])
                candidates=q[(slots==slot)&((residual>=self.limit)|
                    (q[:,2]-low[slot]>=self.limit))]
                # 観測した危険範囲を小voxel代表点で保持。上限超過を診断する。解除は窓の観測有効性に従う。
                for point in candidates:
                    if all(np.linalg.norm(point-old)>self.witness_spacing for old in state['witnesses']):
                        if len(state['witnesses'])>=8:
                            state['overflow']=True;self.counters['overflow']+=1;break
                        state['witnesses'].append(point.copy())
            elif not state['active'] and known.all() and count[slot]>=self.min_points:
                state['obstacle']=0
        # 各受理画像につき一つの窓要素。欠測を0mには変換しない。
        for key,state in self.cells.items():
            if not state['active']:continue
            ix,iy=np.asarray(key)-origin;slot=int(iy*self.width+ix)
            if danger[slot]:continue
            good=(count[slot]>=max(self.min_points,int(np.ceil(self.count_ratio*state['points'])))
                  and np.isfinite(maximum[slot])
                  and abs(low[slot]-state['ground'])<=self.ground_tolerance
                  and all(np.isfinite(f[k][iy,ix]) for k in ('slope_deg','roughness','step_height')))
            current=max(0.,float(maximum[slot])) if good else None
            state['lows'].append(current)
            if len(state['lows'])<self.window:continue
            valid=[v for v in state['lows'] if v is not None]
            if not valid or np.mean(valid)<self.limit*.995:
                state.update(active=False,obstacle=0 if good else -1,
                             confirmed=False,hits=0,witnesses=[])
                self.counters['cleared']+=1
        return len(np.flatnonzero(count))

    def costs(self):
        """同一originの独立cost配列。未知セル・領域外を0で埋めない。"""
        obstacle=np.full((self.height,self.width),-1,dtype=np.int8)
        terrain=obstacle.copy();origin=np.rint(self.origin/self.res).astype(int)
        for key,state in self.cells.items():
            x,y=np.asarray(key)-origin;obstacle[y,x]=state['obstacle'];terrain[y,x]=state['terrain']
        return obstacle,terrain
