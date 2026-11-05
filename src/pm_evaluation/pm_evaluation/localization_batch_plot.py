"""共通ENUへ固定配置し、GNSS状態・起終点・欠測を明示して描画する。"""
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from pyproj import Geod, Transformer

from pm_evaluation.localization_batch_io import messages, header_sec, TOPICS
from pm_evaluation.cli.plot_bag_trajectories import GNSS_STATE_STYLE, infer_rtk_state_from_msg
from pm_localization.vio_odom_adapter_node import quat_to_rot


def load(bag, topics=TOPICS):
    result={}
    for ns,topic,msg in messages(bag,topics):
        result.setdefault(topic,[]).append((ns,msg))
    for rows in result.values():rows.sort(key=lambda r:r[0])
    return result


def closest(rows, stamp):
    """headerで近傍を選ぶ。状態の鮮度判定は呼出元で行う。"""
    if not rows:return None
    times=[header_sec(m) for _,m in rows]
    i=int(np.searchsorted(times,stamp));candidates=rows[max(0,i-1):min(len(rows),i+1)]
    return min(candidates,key=lambda r:abs(header_sec(r[1])-stamp))


def fixes(data):
    nav=data.get('/navpvt',[]);nt=np.array([n for n,_ in nav],dtype=np.int64)
    geod=Geod(ellps='WGS84');points=[];origin=None
    for ns,m in data.get('/fix',[]):
        if m.status.status<0 or not all(math.isfinite(v) for v in (m.latitude,m.longitude)):
            continue
        if not -90<=m.latitude<=90 or not -180<=m.longitude<=180:continue
        if origin is None:origin=(m.longitude,m.latitude)
        az,_,d=geod.inv(*origin,m.longitude,m.latitude)
        state='UNKNOWN'
        if nav:
            i=int(np.searchsorted(nt,ns));j=min(range(max(0,i-1),min(len(nav),i+1)),key=lambda k:abs(int(nt[k])-ns))
            if abs(int(nt[j])-ns)<=2000000000:state=infer_rtk_state_from_msg(nav[j][1])
        points.append((header_sec(m),d*math.sin(math.radians(az)),d*math.cos(math.radians(az)),m.altitude,state))
    return points,origin


def map_cache(points, origin, path, supplied=None, no_map=False):
    """地図取得失敗を隠さず返す。キャッシュ範囲外なら別試行の地図を流用しない。"""
    if not origin:return None,'GNSSなし'
    merc=Transformer.from_crs('EPSG:4326','EPSG:3857',always_xy=True)
    lonlat=Transformer.from_crs('EPSG:3857','EPSG:4326',always_xy=True)
    geod=Geod(ellps='WGS84')
    def extent_local(bounds):
        out=[]
        for x,y in [(bounds[0],bounds[2]),(bounds[1],bounds[3])]:
            lon,lat=lonlat.transform(x,y);az,_,d=geod.inv(*origin,lon,lat)
            out.append((d*math.sin(math.radians(az)),d*math.cos(math.radians(az))))
        return [out[0][0],out[1][0],out[0][1],out[1][1]]
    candidate=Path(supplied) if supplied else path
    if candidate.exists():
        with np.load(candidate) as z:image=z['image'].copy();extent=extent_local(z['extent'])
        xy=np.array([p[1:3] for p in points])
        if (xy[:,0].min()>=extent[0] and xy[:,0].max()<=extent[1] and
            xy[:,1].min()>=extent[2] and xy[:,1].max()<=extent[3]):
            if supplied and candidate.resolve()!=path.resolve():
                import shutil
                shutil.copy2(candidate,path)
            return (image,extent),'cache'
        if supplied:raise ValueError('指定地図はGNSS経路を覆っていない')
    if no_map:return None,'--no-map指定'
    try:
        import contextily as ctx
        x,y=merc.transform(*origin);xy=np.array([p[1:3] for p in points]);scale=1/math.cos(math.radians(origin[1]))
        margin=30*scale
        image,extent=ctx.bounds2img(x+xy[:,0].min()*scale-margin,y+xy[:,1].min()*scale-margin,
                                  x+xy[:,0].max()*scale+margin,y+xy[:,1].max()*scale+margin,
                                  zoom=17,source=ctx.providers.OpenStreetMap.Mapnik)
        np.savez_compressed(path,image=image,extent=extent)
        return (image,extent_local(extent)),'downloaded'
    except Exception as exc:
        return None,'地図取得失敗: '+str(exc)


def pose_series(rows):
    result=[]
    for _,m in rows:
        p=m.pose.pose.position;v=m.twist.twist.linear
        row=[header_sec(m),p.x,p.y,p.z,v.x,v.y,v.z,
             m.pose.covariance[0],m.pose.covariance[7],m.pose.covariance[14],
             m.twist.covariance[0],m.twist.covariance[7],m.twist.covariance[14]]
        if all(math.isfinite(x) for x in row):result.append(row)
    return np.array(sorted(result,key=lambda x:x[0]),dtype=float).reshape((-1,13))


def placement(data, gps_points):
    """地図へ配置する一定offsetとmap←odomを得る。GNSS軌跡へのfitはしない。"""
    offset=None;reference=None
    gps=data.get('/odometry/gps',[])
    for p in gps_points:
        row=closest(gps,p[0])
        if row and abs(header_sec(row[1])-p[0])<.3:
            pos=row[1].pose.pose.position
            offset=np.array([p[1]-pos.x,p[2]-pos.y]);reference=p[0];break
    transforms=[]
    for _,m in data.get('/tf',[]):
        for t in m.transforms:
            if t.header.frame_id=='map' and t.child_frame_id=='odom':
                q=t.transform.translation
                transforms.append((header_sec(t),quat_to_rot(t.transform.rotation),np.array([q.x,q.y,q.z])))
    if reference is None:return None,'GNSSと/odometry/gpsの対応なし。ENU配置を推測しない'
    if not transforms:return None,'map→odomなし。ENU配置を推測しない'
    tf=min(transforms,key=lambda t:abs(t[0]-reference))
    if abs(tf[0]-reference)>.3:return None,'基準時刻のmap→odomがない'
    return (offset,tf[1],tf[2],reference),'同時刻GPS offsetと初回固定map→odom。回転fitなし'


def mapped(rows, transform, frame):
    a=pose_series(rows)
    if not len(a):return a
    offset,r,t,reference=transform
    a=a[a[:,0]>=reference]
    if frame=='odom':a[:,1:4]=a[:,1:4]@r.T+t
    elif frame!='map':raise ValueError('非対応pose frame: '+frame)
    a[:,1:3]+=offset
    return a


def _line(ax,a,x,y,label,**style):
    """大きなheader空白を直線で補間しない。"""
    if not len(a):return
    gap=np.flatnonzero(np.diff(a[:,0])>1.)+1
    for i,part in enumerate(np.split(a,gap)):
        ax.plot(part[:,x],part[:,y],label=label if i==0 else None,**style)


def figure_notes(data, expected=(), reference=(), context=()):
    """記録から確認できる欠落・停止・診断のみ注記し、未記録を故障と断定しない。

    expectedは当該構成で必要なtopicだけ。停止判定は基準系列より末尾が2秒以上
    短い場合、空白はheader間隔が1秒と正の間隔中央値の3倍を超える場合とする。
    いずれも故障原因の断定ではない。
    """
    notes=list(context)
    end=max((header_sec(m) for topic in reference for _,m in data.get(topic,[])),default=None)
    for topic in expected:
        rows=data.get(topic,[])
        if not rows:
            notes.append('INPUT NOT OBSERVED: '+topic+' (missing/unrecorded)');continue
        times=np.array([header_sec(m) for _,m in rows])
        speeds=[]
        for _,m in rows:
            if hasattr(m,'twist') and hasattr(m.twist,'twist'):
                v=m.twist.twist.linear;speeds.append(float(np.linalg.norm([v.x,v.y,v.z])))
        if speeds:
            if not np.isfinite(speeds).all():notes.append('INPUT INVALID: '+topic+' (non-finite velocity)')
            finite=[v for v in speeds if np.isfinite(v)]
            if finite and max(finite)>3:
                notes.append('INPUT SUSPECT: '+topic+' (3D speed max %.2f m/s > 3 m/s)'%max(finite))
        if end is not None and end-times.max()>2:
            notes.append('INPUT ENDS EARLY: '+topic+' (%.1f s before reference)'%(end-times.max()))
        # 1 Hz GNSSの通常の時刻揺れを欠測扱いしない。正の間隔中央値の3倍も要求する。
        gaps=np.diff(times);positive=gaps[gaps>0]
        limit=max(1.,3*float(np.median(positive))) if len(positive) else 1.
        if len(gaps) and gaps.max()>limit:
            notes.append('INPUT GAP: '+topic+' (max %.2f s)'%np.diff(times).max())
    for topic in ('/ov_msckf/diagnostics/health','/vio/adapter/diagnostics',
                  '/vio/twist_gate/diagnostics','/vio/vertical_gate/diagnostics'):
        rows=data.get(topic,[])
        states=[]
        for _,msg in rows:
            for status in msg.status:
                level=status.level if isinstance(status.level,int) else int.from_bytes(status.level,'little')
                if level:
                    text=topic+': '+status.message
                    if text not in states:states.append(text)
        # 状態通知数を欠落数/通過率として扱わず、観測された異常状態の種類を示す。
        notes.extend('DIAGNOSTIC: '+x for x in states)
    return list(dict.fromkeys(notes))


def annotate_figure(fig, notes):
    """全PNGへ同じ注記欄を確保する。詳細は隣接JSONへ保存する。"""
    import textwrap
    lines=[]
    for note in notes or ['No missing-input alert detected; input health is not guaranteed.']:
        lines.extend(textwrap.wrap(note, width=115))
    # 多数の診断理由で図を覆わず、全文の保存先を明示する。
    if len(lines)>12:lines=lines[:11]+['More details: plot_annotations.json']
    height=min(.45,.035+.022*len(lines))
    fig.tight_layout(rect=(0,height,1,1))
    # 診断nodeのmessageには日本語もある。既存CJKフォントがあれば注記欄だけに使う。
    from matplotlib.font_manager import FontProperties
    fonts=list(Path('/usr/share/fonts/opentype/noto').glob('NotoSansCJK-Regular.ttc'))
    font=FontProperties(fname=str(fonts[0])) if fonts else None
    fig.text(.02,.015,'\n'.join(lines),va='bottom',fontsize=8,fontproperties=font,
             color='darkred' if notes else 'dimgray',
             bbox=dict(facecolor='white',edgecolor='gray',alpha=.95))


def gnss_view_limits(gps):
    """全GNSS点を含む外接範囲へ5%（最低1 m）の余白を付ける。

    地図tileの広いextentへautoscaleさせず、軌跡を大きく見せる。
    外れたGNSS点も表示範囲へ含め、見栄えのための点の除去は行わない。
    """
    xy=np.asarray([p[1:3] for p in gps],dtype=float)
    minimum=xy.min(axis=0);maximum=xy.max(axis=0)
    margin=max(1.,float(np.ptp(xy,axis=0).max())*.05)
    return [float(minimum[0]-margin),float(maximum[0]+margin),
            float(minimum[1]-margin),float(maximum[1]+margin)]


def draw(folder, gps, series, basemap, title, notes=None):
    """PNG/CSV・共通時刻GNSS差を出力。GNSSは真値ではない。"""
    folder.mkdir(parents=True,exist_ok=True)
    notes=list(notes or [])
    if not gps:notes.append('PLOT MISSING: GNSS (no valid fixes)')
    for name,a in series.items():
        if not len(a):notes.append('PLOT MISSING: '+name+' (no drawable samples)')
    (folder/'plot_annotations.json').write_text(json.dumps(notes,ensure_ascii=False,indent=2))
    if gps:
        with (folder/'gnss.csv').open('w') as f:
            w=csv.writer(f);w.writerow(['stamp','east','north','altitude','state']);w.writerows(gps)
    for name,a in series.items():
        with (folder/(name+'.csv')).open('w') as f:
            w=csv.writer(f);w.writerow(['stamp','east','north','z','vx','vy','vz','var_x','var_y','var_z','var_vx','var_vy','var_vz']);w.writerows(a)
    fig,ax=plt.subplots(figsize=(10,9))
    if basemap:ax.imshow(basemap[0],extent=basemap[1],origin='upper')
    else:ax.text(.01,.01,'NO BASEMAP',transform=ax.transAxes,color='red')
    if gps:
        for state,style in GNSS_STATE_STYLE.items():
            selected=[p for p in gps if p[4]==state]
            if selected:ax.scatter([p[1] for p in selected],[p[2] for p in selected],s=12,
                                  c=style['color'],marker=style['marker'],label=style['label'])
        g=np.array([p[:4] for p in gps]);_line(ax,g,1,2,'GNSS',color='gray',alpha=.5)
        ax.scatter(g[0,1],g[0,2],marker='^',s=130,c='limegreen',edgecolors='black',label='START')
        ax.scatter(g[-1,1],g[-1,2],marker='X',s=130,c='red',edgecolors='black',label='GOAL')
        ax.annotate('START',g[0,1:3],xytext=(8,15),textcoords='offset points')
        ax.annotate('GOAL',g[-1,1:3],xytext=(-40,-20),textcoords='offset points')
        for fraction in (.2,.4,.6,.8):
            i=int((len(g)-1)*fraction);j=min(i+3,len(g)-1)
            if g[j,0]-g[i,0]<=5 and np.linalg.norm(g[j,1:3]-g[i,1:3])>1:
                ax.annotate('',g[j,1:3],g[i,1:3],arrowprops=dict(arrowstyle='->',color='black'))
    colors={'local':'tab:blue','global':'tab:red','vio':'green','wheel_imu':'tab:orange'}
    for name,a in series.items():_line(ax,a,1,2,name,color=colors.get(name))
    ax.set(title=title,xlabel='East [m]',ylabel='North [m]');ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2)
    ax.legend(fontsize=8);ax.text(.99,.01,'© OpenStreetMap contributors',ha='right',transform=ax.transAxes,fontsize=8)
    # GNSS単体/全推定欠落の図だけは、広いtile全体ではなく全ログ範囲を拡大する。
    # 推定の発散を含む通常の全体比較図は、従来どおり全系列の表示範囲を維持する。
    gnss_only=bool(gps) and not any(len(a) for a in series.values())
    if gnss_only:
        limits=gnss_view_limits(gps);ax.set_xlim(limits[:2]);ax.set_ylim(limits[2:])
        (folder/'view_extent.json').write_text(json.dumps(dict(extent_m=limits,scope='all valid GNSS fixes; 5% margin, minimum 1 m'),indent=2))
    annotate_figure(fig,notes);fig.savefig(folder/'xy_full.png',dpi=140)
    if gps:
        if not gnss_only:
            xy=np.array([p[1:3] for p in gps]);margin=max(15.,np.ptp(xy,axis=0).max()*.1)
            ax.set_xlim(xy[:,0].min()-margin,xy[:,0].max()+margin);ax.set_ylim(xy[:,1].min()-margin,xy[:,1].max()+margin)
        fig.savefig(folder/'xy_gnss_area.png',dpi=140)
    plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(11,8),sharex=True)
    starts=[a[0,0] for a in series.values() if len(a)]+([gps[0][0]] if gps else [])
    start=min(starts) if starts else 0
    for name,a in series.items():
        if len(a):
            axes[0].plot(a[:,0]-start,a[:,3],label=name)
            axes[1].plot(a[:,0]-start,a[:,3]-a[0,3],label=name)
    if gps:
        g=np.array([p[:4] for p in gps]);axes[0].plot(g[:,0]-start,g[:,3],label='GNSS altitude',color='gray')
        axes[1].plot(g[:,0]-start,g[:,3]-g[0,3],label='GNSS altitude change',color='gray')
    for ax in axes:ax.grid();ax.legend(fontsize=8)
    axes[0].set_ylabel('z / raw altitude [m]');axes[1].set(ylabel='height change [m]',xlabel='time [s]')
    axes[0].set_title('Raw heights use different datums; changes are separate')
    annotate_figure(fig,notes);fig.savefig(folder/'z.png',dpi=140);plt.close(fig)
    metrics={}
    # 全系列の共通区間だけを評価し、途絶を越える補間を避ける。
    nonempty=[a for a in series.values() if len(a)]
    begin=max([a[0,0] for a in nonempty],default=0);end=min([a[-1,0] for a in nonempty],default=0)
    for name,a in series.items():
        distances=[]
        if len(a):
            for p in gps:
                i=int(np.searchsorted(a[:,0],p[0]))
                if begin<=p[0]<=end and 0<i<len(a) and a[i,0]-a[i-1,0]<=1:
                    d=np.array([np.interp(p[0],a[:,0],a[:,k]) for k in (1,2)])-p[1:3]
                    distances.append(float(np.linalg.norm(d)))
        metrics[name]={'count':len(a),'common_gnss_pairs':len(distances),
                       'gnss_distance_median_m':float(np.median(distances)) if distances else None,
                       'gnss_distance_p95_m':float(np.percentile(distances,95)) if distances else None,
                       'note':'GNSSは真値ではない。fitなし'}
    (folder/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n')
    return metrics
