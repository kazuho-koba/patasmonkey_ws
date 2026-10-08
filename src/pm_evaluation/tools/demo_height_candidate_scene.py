#!/usr/bin/env python3
"""高さ候補・短期平均・確認／解除を説明する自己完結HTMLを生成する。

--syntheticは仕組み説明用の人工点列、bag指定は実depthとmapper実使用TF。
認識に使うsamplingを勝手に増減しない。表示点だけ別途上限で間引く。
通常node、センサ、モータ、bag再生プロセスは一切起動しない。
"""
import argparse
from collections import defaultdict, deque
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import yaml

from pm_evaluation.height_candidates import CandidateOptions, extract_candidates
from pm_evaluation.height_candidate_tracks import CandidateTracks
from pm_evaluation.obstacle_height_window import ObstacleHeightWindow
from pm_evaluation.height_demo_html import write_demo_html


class SceneBuilder:
    """表示snapshot間のframeも更新する。候補対応・解除規則は既存診断と同じ。"""

    def __init__(self, resolution, limits, options, view_radius, max_points):
        self.resolution = resolution
        self.limits = np.asarray(limits)
        self.options = options
        self.view_radius, self.max_points = view_radius, max_points
        self.tracker = CandidateTracks(5, .05, .03)
        self.histories = defaultdict(lambda: deque(maxlen=5))
        self.stores = {n: {} for n in (1, 3, 5)}
        self.terrain, self.display_z = {}, {}

    def update(self, points, pixels, pose, stamp, index, baseline, planes, usable):
        """現frame候補を分類し、保持hazardを更新。snapshot用の中間値を返す。"""
        # bag modeは既存診断の集約をそのまま使う。人工例も同じ式で高さを計算する。
        height = maximum_residuals(points, baseline['origin'], baseline['hazard'].shape,
                                   self.resolution, planes, usable)
        candidates, layers = extract_candidates(points, pixels, baseline['origin'],
            baseline['hazard'].shape, self.resolution, planes, usable, self.limits[3], self.options)
        origin = np.rint(baseline['origin']/self.resolution).astype(int)
        width = baseline['hazard'].shape[1]
        groups = defaultdict(list)
        for row in candidates:
            slot = row['slot']; cell = (int(origin[0]+slot % width), int(origin[1]+slot//width))
            row['high_hit'] = bool(row['max_residual_m'] >= self.limits[3]) if row['plane_usable'] else None
            groups[cell].append(row)
        objects = []; clears = {str(n): 0 for n in self.stores}
        for slot in np.flatnonzero(layers['point_count']):
            cell = (int(origin[0]+slot % width), int(origin[1]+slot//width))
            yy, xx = divmod(int(slot), width)
            # 元frameのcell最小zを表示アンカーとして保持する。hazardの図形の高さであり
            # 新しく高さ融合・地面mesh補間した結果ではない。
            self.display_z[cell] = float(baseline['cell_min'][yy, xx])
            cues = np.array([baseline[name][yy, xx] for name in ('slope_deg','roughness','step_height')])
            if np.isfinite(cues).all():
                self.terrain[cell] = cues.tolist()
            tracked = self.tracker.update(cell, groups[cell], stamp, index)
            histories = []
            for row, tracking in zip(groups[cell], tracked):
                history = self.histories[tracking['track_id']]
                history.append((stamp, row['high_hit'], row['z_mean_m']))
                histories.append(list(history))
                objects.append(dict(cell=list(cell), kind=row['kind'], count=int(row['count']),
                    bounds=[row[k] for k in ('x_lo_m','x_hi_m','y_lo_m','y_hi_m','z_lo_m','z_hi_m')],
                    track_id=tracking['track_id'], high_hit=row['high_hit'], plane_usable=row['plane_usable'],
                    mean_z={str(n): float(np.mean([v[2] for v in list(history)[-n:]])) for n in self.stores},
                    history_size=len(history), positive_support=sum(v[1] is True for v in history)))
            if not np.isfinite(height[slot]):
                continue
            # 描画用集約も既存候補抽出の全点hitと一致していることを確認する。
            assert bool(height[slot] >= self.limits[3]) == bool(layers['above_plane_hit'][yy, xx])
            for n, store in self.stores.items():
                state = store.setdefault(cell, ObstacleHeightWindow(n, self.limits[3]))
                onset = state.onset if state.active else stamp
                confirmed = any(len(h) >= n and all(s >= onset and value is True for s, value, _ in h[-n:]) for h in histories)
                cleared = state.update(float(height[slot]), stamp, confirmed)
                clears[str(n)] += int(cleared is not None)
        return objects, clears, int(np.count_nonzero(usable))

    def snapshot(self, points, pose, stamp, index, seconds, objects, clears, valid_cells):
        """表示半径・点数上限は認識状態に影響させず、書出しだけに適用する。"""
        res = self.resolution
        nearby = lambda x, y: np.hypot(x-pose[1], y-pose[2]) <= self.view_radius
        selected = points[np.hypot(points[:, 0]-pose[1], points[:, 1]-pose[2]) <= self.view_radius]
        if len(selected) > self.max_points:
            selected = selected[np.linspace(0, len(selected)-1, self.max_points, dtype=int)]
        tiles, counts = {}, {}
        for n, store in self.stores.items():
            tile_rows = []; pending = confirmed = 0
            for cell, z in self.display_z.items():
                x, y = (cell[0]+.5)*res, (cell[1]+.5)*res
                if not nearby(x, y):
                    continue
                state = store.get(cell)
                obstacle = state.value if state else np.nan
                values = np.array(self.terrain.get(cell, [np.nan]*3)+[obstacle])
                known = np.isfinite(values); scores = np.clip(values/self.limits, 0., 1.)
                cost = int(np.rint(np.nanmax(scores)*100)) if known.any() else None
                causes = '+'.join(name for name, score in zip(('slope','roughness','step','obstacle'), scores)
                                  if np.isfinite(score) and np.rint(score*100) >= 100)
                label = 'confirmed' if state and state.active and state.confirmed else 'pending' if state and state.active else 'inactive'
                pending += label == 'pending'; confirmed += label == 'confirmed'
                tile_rows.append([x, y, z, cost, int((~known).sum()), label, causes,
                                  float(obstacle) if np.isfinite(obstacle) else None])
            tiles[str(n)] = tile_rows
            counts[str(n)] = dict(pending=pending, confirmed=confirmed)
        return dict(stamp_ns=stamp, seconds=seconds, frame_index=index,
            pose=[pose[1],pose[2],pose[3],pose[-1]], points=selected.tolist(), full_point_count=len(points),
            objects=[o for o in objects if nearby((o['bounds'][0]+o['bounds'][1])/2,
                                                  (o['bounds'][2]+o['bounds'][3])/2)],
            tiles=tiles, states=counts, clears=clears, valid_cells=valid_cells)


def maximum_residuals(points, origin, shape, resolution, planes, usable):
    """全点の最大暫定平面残差[m]。候補枠上限により高点を落とさない。"""
    h, w = shape
    index = np.floor((points[:, :2]-origin)/resolution).astype(int)
    inside = (index[:, 0] >= 0) & (index[:, 0] < w) & (index[:, 1] >= 0) & (index[:, 1] < h)
    index, selected = index[inside], points[inside]
    slots = index[:, 1]*w+index[:, 0]
    coefficients = planes.reshape(-1, 3)[slots]
    centers = origin+(index+.5)*resolution
    residual = selected[:, 2]-(coefficients[:, :2]*(selected[:, :2]-centers)).sum(axis=1)-coefficients[:, 2]
    result = np.full(h*w, -np.inf); np.maximum.at(result, slots, residual)
    result[(~usable.reshape(-1)) | (~np.isfinite(result))] = np.nan
    return np.maximum(result, 0.)


def synthetic_frames():
    """人工例：地面、縦に広い構造、高い薄い群、持続高点、消える高点。

    実センサの性能を示す例ではない。平面z=0を既知として与えることで、
    表現・窓更新の説明だけを再現可能にする。枝の通過可否は判定しない。
    """
    resolution = .1; shape = (40, 40); origin = np.array([-1., -2.])
    for index in range(16):
        ground = np.array([[x,y,0.] for x in np.arange(-.95,2.96,.1) for y in np.arange(-1.95,1.96,.1)])
        wall = np.array([[1.23,y,z] for y in np.arange(.03,.44,.1) for z in np.arange(.03,1.24,.03)])
        branch = np.array([[x,.53,1.20+delta] for x in np.arange(1.53,2.34,.1) for delta in (0.,.015)])
        persistent = np.array([[.83,-.43,.12],[.84,-.42,.13]])
        noise = np.array([[1.03,-.53,.18]]) if index < 3 else np.empty((0,3))
        points = np.vstack((ground,wall,branch,persistent,noise))
        minimum = np.full(shape[0]*shape[1], np.inf)
        idx = np.floor((points[:, :2]-origin)/resolution).astype(int)
        np.minimum.at(minimum, idx[:,1]*shape[1]+idx[:,0], points[:,2]); minimum[~np.isfinite(minimum)] = np.nan
        baseline = dict(origin=origin,hazard=np.zeros(shape),cell_min=minimum.reshape(shape),
                        slope_deg=np.zeros(shape),roughness=np.zeros(shape),step_height=np.zeros(shape))
        planes = np.zeros(shape+(3,)); usable = np.ones(shape,dtype=bool)
        pose = (index*100000000, 0.,0.,.12,0.,0.,0.)
        yield index, points, np.column_stack((np.arange(len(points)),np.zeros(len(points)))), pose, baseline, planes, usable


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', nargs='?'); parser.add_argument('--mapper-trace')
    parser.add_argument('--synthetic', action='store_true')
    parser.add_argument('--output', required=True, help='新規HTML。既存出力は上書きしない')
    parser.add_argument('--config', default=str(Path(__file__).resolve().parents[1]/'config/height_candidate_diagnosis.yaml'))
    parser.add_argument('--display-every-seconds',type=float,default=1.)
    parser.add_argument('--start-seconds',type=float,default=0.)
    parser.add_argument('--max-display-frames',type=int,default=60)
    parser.add_argument('--view-radius',type=float,default=4.)
    parser.add_argument('--max-display-points',type=int,default=1800)
    args = parser.parse_args()
    if (args.synthetic and (args.bag or args.mapper_trace)) or (not args.synthetic and not (args.bag and args.mapper_trace)):
        parser.error('--synthetic または bag と --mapper-trace の組を指定してください')
    if (not all(np.isfinite(v) for v in (args.display_every_seconds,args.start_seconds,args.view_radius))
            or args.display_every_seconds <= 0 or args.start_seconds < 0 or args.view_radius <= 0
            or args.max_display_frames < 1 or args.max_display_points < 1):
        parser.error('表示周期・半径・上限は正値、start-secondsは非負値です')
    output = Path(args.output)
    if output.exists() or output.with_suffix('.json').exists():
        parser.error('既存HTML／JSONを上書きしません。別のoutputを指定してください')
    options = CandidateOptions(**yaml.safe_load(Path(args.config).read_text()))
    if args.synthetic:
        params = dict(resolution=.1,hazard_slope_limit_deg=20.,hazard_roughness_limit=.03,
                      hazard_step_limit=.07,hazard_obstacle_height_limit=.08)
        iterator = synthetic_frames()
        metadata = dict(source='人工説明例（実測結果ではありません）',synthetic=True)
    else:
        # ROS／MCAP依存は実bag読出し時だけ読み、人工デモはROSなしでも生成可能にする。
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image
        from diagnose_height_candidates import prepare_frame
        from evaluate_single_depth_terrain import messages, stamp_ns
        from pm_evaluation.height_candidates import plane_reasons
        from pm_perception.mapper_replay_trace import load_mapper_trace
        meta, frames, _ = load_mapper_trace(args.mapper_trace); params = meta['parameters']

        def bag_frames():
            index = 0
            for _, _, record in messages(args.bag,[params['depth_topic']]):
                depth = deserialize_message(record.data,Image); stamp = stamp_ns(depth)
                if stamp not in frames:
                    continue
                points, _, u, v, pose, baseline, planes = prepare_frame(depth,frames[stamp],params)
                yield index,points,np.column_stack((u,v)),pose,baseline,planes,plane_reasons(baseline,params)['plane_usable']
                index += 1
        iterator = bag_frames()
        metadata = dict(source='bag: '+str(args.bag),synthetic=False,trace=str(args.mapper_trace),
                        trace_sha256=hashlib.sha256(Path(args.mapper_trace).read_bytes()).hexdigest())
    limits = [params[k] for k in ('hazard_slope_limit_deg','hazard_roughness_limit','hazard_step_limit','hazard_obstacle_height_limit')]
    scene = SceneBuilder(params['resolution'],limits,options,args.view_radius,args.max_display_points)
    snapshots = []; first_stamp = last_display = None; began = time.perf_counter(); processed = 0
    for index,points,pixels,pose,baseline,planes,usable in iterator:
        stamp = int(pose[0]); first_stamp = stamp if first_stamp is None else first_stamp
        seconds = (stamp-first_stamp)/1e9
        objects,clears,valid = scene.update(points,pixels,pose,stamp,index,baseline,planes,usable)
        processed += 1
        display = args.synthetic or (seconds >= args.start_seconds and
                                    (last_display is None or (stamp-last_display)/1e9 >= args.display_every_seconds))
        if display:
            snapshots.append(scene.snapshot(points,pose,stamp,index,seconds,objects,clears,valid));last_display=stamp
            if len(snapshots) >= args.max_display_frames:
                break
        if processed % 100 == 0:
            print('processed %d adopted frames' % processed,flush=True)
    if not snapshots:
        raise ValueError('表示対象がありません。start-seconds等を確認してください')
    metadata.update(parameters=params,candidate_options=vars(options),resolution=params['resolution'],
                    view_radius_m=args.view_radius,processed_adopted_frames=processed,
                    snapshot_count=len(snapshots),elapsed_seconds=time.perf_counter()-began,
                    display_interval_seconds=args.display_every_seconds,
                    robot_proxy='幅0.45m、長さ0.55m、高さ0.22mの目安。実URDFではない',
                    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    limitations=['表示snapshotは間引くが、その間の採用frameも状態更新する',
                                 '箱は現frameの観測外包。保持hazardは別の情報',
                                 'unknown／解除は安全の証明ではない',
                                 '全高未設定：高い枝・天井を通過可能とは分類しない'])
    document = dict(metadata=metadata,frames=snapshots)
    output.parent.mkdir(parents=True,exist_ok=True)
    write_demo_html(output,document)
    output.with_suffix('.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
    print('saved %s (%d snapshots, %.2f MiB)' % (output,len(snapshots),output.stat().st_size/2**20),flush=True)


if __name__ == '__main__':
    main()
