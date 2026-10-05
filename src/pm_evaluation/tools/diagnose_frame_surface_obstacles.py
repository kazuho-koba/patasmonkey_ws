#!/usr/bin/env python3
"""仮説②と方式Eを同一frameで比較するオフライン診断。

通常のterrain指標・TF・採用画素は固定し、obstacle候補だけ交換する。地面候補は
セルzの10%分位点を3×3近傍で平面fitしたもの。障害物面を地面としてfitする
危険もあるため、候補値は安全判定の正解ではない。通常perceptionは変更しない。
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np
import evaluate_spatial_feature_coverage as evaluation
from pm_perception.terrain_features import compute_terrain_features


def grouped_statistics(slots, values, size):
    """cellごとにsortし、NumPy標準の線形補間q10/q90と両極値の元indexを返す。

    全cellのPython loopや3D voxelは不要。各点の元配列indexを維持して元画素へ
    追跡する。同一frameの多数pixelは独立証拠ではない。
    """
    order = np.lexsort((values, slots))
    count = np.bincount(slots, minlength=size)
    start = np.cumsum(count)-count
    valid = count > 0
    result = {}
    for name, q in (('q10', .1), ('q90', .9)):
        location = (count[valid]-1)*q
        lo = np.floor(location).astype(int); hi = np.ceil(location).astype(int)
        out = np.full(size, np.nan)
        out[valid] = (values[order[start[valid]+lo]]*(hi-location)
                      + values[order[start[valid]+hi]]*(location-lo))
        # 整数rankでは上の両重みが0となるため元の値を明示的に採る。
        integer = lo == hi
        out[np.flatnonzero(valid)[integer]] = values[order[start[valid][integer]+lo[integer]]]
        result[name] = out
    for name, rank in (('min_index', start), ('max_index', start+count-1)):
        out = np.full(size, -1, dtype=int)
        out[valid] = order[rank[valid]]
        result[name] = out
    result['count'] = count
    return result


def main():
    def option(name, default=None):
        return sys.argv[sys.argv.index(name)+1] if name in sys.argv else default
    output = Path(option('--output'))
    meta, frames, _ = evaluation.load_mapper_trace(option('--mapper-trace'))
    params = meta['parameters']; limits = np.array([params[k] for k in
        ('hazard_slope_limit_deg', 'hazard_roughness_limit', 'hazard_step_limit', 'hazard_obstacle_height_limit')])
    modes = ('quantile_span', 'detrended_span', 'surface_q90')
    path = []; deadlines = {}
    for stamp, frame in frames.items():
        base = frame['base_to_map']
        pose = evaluation.pose_for_footprint(stamp, base['translation']+base['quaternion'])
        if not path or np.hypot(pose[1]-path[-1][1], pose[2]-path[-1][2]) >= float(option('--path-spacing', .25)):
            path.append(pose)
    for pose in path:
        for cell in evaluation.footprint_cells(pose, params['resolution'], .45, .55):
            deadlines[cell] = max(deadlines.get(cell, -1), int(pose[0]))
    originals = (evaluation.sampled_points, evaluation.evaluate_points, evaluation.SpatialFeatureStore)
    state = {}; evidence = []; passage_rows = {mode: [] for mode in modes}

    def sampled(*args, **kwargs):
        points, u, v, depth = originals[0](*args, **kwargs, return_pixels=True)
        message = args[0]
        raw = np.frombuffer(message.data, dtype='>u2' if message.is_bigendian else '<u2').reshape(
            message.height, message.step//2)[:, :message.width]
        state.update(u=u, v=v, depth=depth, raw=raw)
        return points

    def evaluated(points, p, stamp, **kwargs):
        arrays = originals[1](points, p, stamp, **kwargs)
        h, w = arrays['hazard'].shape; size = h*w; res = p['resolution']
        ix = np.floor((points[:, 0]-arrays['origin'][0])/res).astype(int)
        iy = np.floor((points[:, 1]-arrays['origin'][1])/res).astype(int)
        inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        selected = np.flatnonzero(inside); xyz = points[inside]; slots = iy[inside]*w+ix[inside]
        stats = grouped_statistics(slots, xyz[:, 2], size)
        # q10のodom-zを局所平面へfit。中心cellからのdx/dy[m]が係数a/bの引数。
        # min-groundではなく低分位点を使うが、物体面の混入・姿勢誤差は残りうる。
        ground = stats['q10'].reshape(h, w)
        features = compute_terrain_features(ground, np.where(np.isfinite(ground), 0., np.nan),
            res, p['feature_max_observation_age'], p['feature_neighborhood_radius_cells'],
            p['feature_min_neighbors'], *limits[:3], include_diagnostics=True)
        a, b, c = [features[k].ravel() for k in ('plane_a', 'plane_b', 'plane_c')]
        dx = xyz[:, 0]-(arrays['origin'][0]+(ix[inside]+.5)*res)
        dy = xyz[:, 1]-(arrays['origin'][1]+(iy[inside]+.5)*res)
        residual = xyz[:, 2]-(a[slots]*dx+b[slots]*dy+c[slots])
        plane_ok = np.isfinite(residual)
        residual_stats = grouped_statistics(slots[plane_ok], residual[plane_ok], size)
        low = np.full(size, np.inf); high = np.full(size, -np.inf)
        np.minimum.at(low, slots[plane_ok], residual[plane_ok])
        np.maximum.at(high, slots[plane_ok], residual[plane_ok])
        detrended = high-low; detrended[residual_stats['count'] == 0] = np.nan
        candidates = dict(quantile_span=stats['q90']-stats['q10'],
                          detrended_span=detrended,
                          surface_q90=np.maximum(residual_stats['q90'], 0))
        # 1点だけで障害物不存在としない。十分なplaneがなければunknownで、0にしない。
        for value in candidates.values():
            value[stats['count'] < 2] = np.nan
        state.update(arrays=arrays, candidates={k:v.reshape(h, w) for k,v in candidates.items()})
        raw_span = (arrays['cell_max']-arrays['cell_min']).ravel()
        black_slots = np.flatnonzero(np.isfinite(raw_span) & (np.rint(np.clip(raw_span/limits[3], 0, 1)*100) >= 100))
        for slot in black_slots:
            x, y = slot % w, slot//w
            cell = (int(np.rint(arrays['origin'][0]/res))+x,
                    int(np.rint(arrays['origin'][1]/res))+y)
            if cell not in deadlines or stamp >= deadlines[cell]:
                continue
            row = dict(stamp_ns=int(stamp), cell_x=cell[0], cell_y=cell[1], count=int(stats['count'][slot]),
                raw_span=float(raw_span[slot]), quantile_span=float(candidates['quantile_span'][slot]),
                detrended_span=float(detrended[slot]), surface_q90=float(candidates['surface_q90'][slot]),
                plane_a=float(a[slot]), plane_b=float(b[slot]), plane_c=float(c[slot]),
                plane_support=float(features['support_count'].ravel()[slot]),
                plane_rms=float(features['roughness'].ravel()[slot]),
                plane_slope_deg=float(features['slope_deg'].ravel()[slot]))
            for label in ('min', 'max'):
                index = selected[stats[label+'_index'][slot]]
                u, v = int(state['u'][index]), int(state['v'][index])
                patch = state['raw'][max(0,v-1):v+2, max(0,u-1):u+2].astype(float)*.001
                valid = patch[(patch >= p['min_depth']) & (patch <= p['max_depth']) & (patch > 0)]
                row.update({label+'_u':u, label+'_v':v, label+'_depth':float(state['depth'][index]),
                    label+'_z':float(points[index,2]), label+'_x':float(points[index,0]), label+'_y':float(points[index,1]),
                    label+'_depth_patch_valid':len(valid),
                    label+'_depth_patch_median':float(np.median(valid)) if len(valid) else np.nan})
            evidence.append(row)
        return arrays

    class ComparisonStore(originals[2]):
        def __init__(self):
            super().__init__(prefer_near=False)
            self.variants = {mode: originals[2](prefer_near=False) for mode in modes}

        def update(self, cells, values, stamp, support, coherent=True):
            # 方式E比較のbaselineは過去結果と同じ最新有効値更新に固定する。
            # 距離優先とobstacle方式の二因子を同時変更しない。
            self.prefer_near = False
            arrays = state['arrays']; h, w = arrays['hazard'].shape
            origin = np.rint(arrays['origin']/params['resolution']).astype(int)
            for mode, store in self.variants.items():
                variant = values.copy()
                for i, cell in enumerate(cells):
                    x, y = cell[0]-origin[0], cell[1]-origin[1]
                    variant[i,3] = state['candidates'][mode][y,x] if 0 <= x < w and 0 <= y < h else np.nan
                store.update(cells, variant, stamp, support, coherent)
            super().update(cells, values, stamp, support, coherent)

        def query(self, cells, now, pose, reference_max_age=None):
            result = super().query(cells, now, pose, reference_max_age)
            if pose is not None and now == int(pose[0])-1 and reference_max_age is None:
                for mode, store in self.variants.items():
                    values = store.query(cells, now, pose)
                    passage_rows[mode].append(dict(passage_stamp_ns=int(pose[0]), **evaluation.describe(values, limits)))
            return result

    evaluation.sampled_points, evaluation.evaluate_points, evaluation.SpatialFeatureStore = sampled, evaluated, ComparisonStore
    try:
        evaluation.main()
    finally:
        evaluation.sampled_points, evaluation.evaluate_points, evaluation.SpatialFeatureStore = originals
    with (output/'black_cell_evidence.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(evidence[0]) if evidence else ['stamp_ns'])
        writer.writeheader(); writer.writerows(evidence)
    baseline = json.loads((output/'summary.json').read_text())
    result = dict(radius_m=baseline['radius_m'], max_depth=baseline['parameters']['max_depth'],
        baseline=dict(path=baseline['path']['before_passage'], causes=baseline['path_causes']['before_passage']),
        modes={k:dict(path=evaluation.aggregate(v), causes=evaluation.cause_summary(v)) for k,v in passage_rows.items()},
        raw_black_observations=len(evidence))
    for mode in modes:
        finite = [r for r in evidence if np.isfinite(r[mode])]
        result.setdefault('raw_black_reclassification', {})[mode] = dict(known=len(finite), unknown=len(evidence)-len(finite),
            still_black=int(sum(np.rint(np.clip(r[mode]/limits[3],0,1)*100) >= 100 for r in finite)))
    for mode, rows in passage_rows.items():
        with (output/(mode+'_passages.csv')).open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    (output/'surface_comparison.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
