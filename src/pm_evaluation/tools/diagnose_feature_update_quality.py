#!/usr/bin/env python3
"""通過セルの観測品質と更新遷移を対照付きで保存する診断専用の入口。

空間保持評価の投影・指標計算・更新規則を再利用し、値の選別は変更しない。
terrain残差を品質順位に変換せず、距離・点数・supportの配置と別々に記録する。
対象は経路footprintの和集合、最後の通過前まで。全画像や全画素履歴は保存しない。
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np
import evaluate_spatial_feature_coverage as evaluation

QUALITY = ("range_m", "cell_pixels", "patch_pixels", "support_cells", "geometry_ratio",
           "centroid_offset_cells", "forward_support", "rear_support", "residual_m", "heading_rad")


def black(value, limit):
    """通常評価と同じ100への丸め。NaNは黒でなくunknownとして分類する。"""
    return bool(np.isfinite(value) and np.rint(np.clip(value/limit, 0, 1)*100) >= 100)


def transition(old, new, limit):
    """known非黒をsafeと略記するが、走行可能性の真値を意味しない。"""
    old_state = "unknown" if not np.isfinite(old) else ("black" if black(old, limit) else "safe")
    return old_state+"_to_"+("black" if black(new, limit) else "safe")


def main():
    """既存mainへの計測hookだけを一時的に装着し、終了時に元へ戻す。"""
    def option(name, default=None):
        return sys.argv[sys.argv.index(name)+1] if name in sys.argv else default
    output = Path(option("--output"))
    meta, frames, _ = evaluation.load_mapper_trace(option("--mapper-trace"))
    params = meta["parameters"]
    spacing = float(option("--path-spacing", .25))
    path = []
    for stamp, frame in frames.items():
        base = frame["base_to_map"]
        pose = evaluation.pose_for_footprint(stamp, base["translation"]+base["quaternion"])
        if not path or np.hypot(pose[1]-path[-1][1], pose[2]-path[-1][2]) >= spacing:
            path.append(pose)
    indices = {int(p[0]): i for i, p in enumerate(path)}
    deadlines = {}
    for pose in path:
        for cell in evaluation.footprint_cells(pose, params["resolution"],
                float(option("--width", .45)), float(option("--length", .55))):
            deadlines[cell] = max(deadlines.get(cell, -1), int(pose[0]))
    limits = [params["hazard_slope_limit_deg"], params["hazard_roughness_limit"],
              params["hazard_step_limit"], params["hazard_obstacle_height_limit"]]
    original_sample, original_evaluate = evaluation.sampled_points, evaluation.evaluate_points
    original_extract, original_store = evaluation.extract, evaluation.SpatialFeatureStore
    state = {}; events = []; passages = []

    def sampled(*args, **kwargs):
        points = original_sample(*args, **kwargs)
        state["ranges"] = np.linalg.norm(points, axis=1)
        return points

    def evaluated(points, p, stamp, **kwargs):
        arrays = original_evaluate(points, p, stamp, **kwargs)
        h, w = arrays["hazard"].shape
        ix = np.floor((points[:, 0]-arrays["origin"][0])/p["resolution"]).astype(int)
        iy = np.floor((points[:, 1]-arrays["origin"][1])/p["resolution"]).astype(int)
        inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        slots = iy[inside]*w+ix[inside]
        sums = np.bincount(slots, weights=state["ranges"][inside], minlength=h*w).reshape(h, w)
        mean = np.full((h, w), np.nan)
        np.divide(sums, arrays["pixel_count"], out=mean, where=arrays["pixel_count"] > 0)
        state.update(arrays=arrays, mean_range=mean, stamp=stamp,
                     heading=kwargs["heading_yaw"], resolution=p["resolution"])
        return arrays

    def extracted(arrays, cells, **kwargs):
        values, support = original_extract(arrays, cells, **kwargs)
        origin = np.rint(arrays["origin"]/state["resolution"]).astype(int)
        h, w = arrays["hazard"].shape
        qualities = {}
        radius = int(params["feature_neighborhood_radius_cells"])
        for cell in cells:
            if cell not in deadlines or state["stamp"] >= deadlines[cell]:
                continue
            x, y = cell[0]-origin[0], cell[1]-origin[1]
            if not (0 <= x < w and 0 <= y < h):
                continue
            offsets = []; pixels = 0; range_sum = 0.
            for dy in range(-radius, radius+1):
                for dx in range(-radius, radius+1):
                    xx, yy = x+dx, y+dy
                    if 0 <= xx < w and 0 <= yy < h and np.isfinite(arrays["relative_elevation"][yy, xx]):
                        offsets.append((dx, dy))
                        n = arrays["pixel_count"][yy, xx]; pixels += n
                        if n:
                            range_sum += state["mean_range"][yy, xx]*n
            if len(offsets) >= 3:
                xy = np.asarray(offsets, dtype=float)
                centroid = xy.mean(axis=0)
                centered = xy-centroid
                eigen = np.linalg.eigvalsh(centered.T@centered/len(xy))
                ratio = eigen[0]/eigen[-1] if eigen[-1] > 0 else 0.
                offset = np.linalg.norm(centroid)
            else:
                ratio = offset = np.nan
            qualities[cell] = dict(range_m=range_sum/pixels if pixels else np.nan,
                cell_range_m=state["mean_range"][y, x], cell_pixels=float(arrays["pixel_count"][y, x]),
                patch_pixels=float(pixels), support_cells=float(arrays["support_count"][y, x]),
                geometry_ratio=float(ratio), centroid_offset_cells=float(offset),
                forward_support=float(arrays["step_support_forward"][y, x]),
                rear_support=float(arrays["step_support_rear"][y, x]),
                residual_m=float(arrays["residual_rms"][y, x]), heading_rad=state["heading"])
        state["qualities"] = qualities
        return values, support

    class DiagnosticStore(original_store):
        """値の更新は親へ委譲し、cueごとの直前品質と黒episodeの起点だけ追加保持する。"""
        def __init__(self):
            super().__init__(); self.quality = {}; self.episodes = {}

        def update(self, cells, values, stamp, support, coherent=True):
            # この診断入口は選別前の全更新を記録する従来対照専用。通常の空間
            # 評価の既定が近距離優先になっても、ログを受理済みと誤記しない。
            self.prefer_near = False
            for cell, row in zip(cells, values):
                if cell not in state["qualities"]:
                    continue
                valid = np.isfinite(row)
                if coherent and not valid[:3].all():
                    valid[:3] = False
                old = self.data[cell][0].copy() if cell in self.data else np.full(4, np.nan)
                old_stamps = self.data[cell][1].copy() if cell in self.data else np.full(4, -1)
                for i in np.flatnonzero(valid):
                    key = (cell, i); current = dict(state["qualities"][cell])
                    # obstacleの距離・画素数は中心セル、terrainは近傍patchを使う。
                    if i == 3:
                        current["range_m"] = current["cell_range_m"]
                    previous = self.quality.get(key, {})
                    kind = transition(old[i], row[i], limits[i])
                    event = dict(cell_x=cell[0], cell_y=cell[1], cue=evaluation.CUES[i],
                        stamp_ns=int(stamp), previous_stamp_ns=int(old_stamps[i]),
                        transition=kind, old_value=float(old[i]), new_value=float(row[i]))
                    for name in QUALITY:
                        event["old_"+name] = previous.get(name, np.nan)
                        event["new_"+name] = current[name]
                    events.append(event)
                    self.quality[key] = current
                    if black(row[i], limits[i]):
                        if not black(old[i], limits[i]):
                            self.episodes[key] = event
                    else:
                        self.episodes.pop(key, None)
            super().update(cells, values, stamp, support, coherent)

        def query(self, cells, now, pose, reference_max_age=None):
            values = super().query(cells, now, pose, reference_max_age)
            if pose is not None and now == int(pose[0])-1 and reference_max_age is None:
                for cell, row in zip(cells, values):
                    for i, value in enumerate(row):
                        if black(value, limits[i]):
                            event = self.episodes.get((cell, i), {})
                            passages.append(dict(path_index=indices[int(pose[0])],
                                passage_stamp_ns=int(pose[0]), cell_x=cell[0], cell_y=cell[1],
                                cue=evaluation.CUES[i], value=float(value),
                                episode_transition=event.get("transition", "missing"),
                                episode_stamp_ns=event.get("stamp_ns", -1),
                                previous_stamp_ns=event.get("previous_stamp_ns", -1),
                                **{k:v for k,v in event.items() if k.startswith(("old_", "new_"))}))
            return values

    evaluation.sampled_points, evaluation.evaluate_points = sampled, evaluated
    evaluation.extract, evaluation.SpatialFeatureStore = extracted, DiagnosticStore
    try:
        evaluation.main()
    finally:
        evaluation.sampled_points, evaluation.evaluate_points = original_sample, original_evaluate
        evaluation.extract, evaluation.SpatialFeatureStore = original_extract, original_store
    for filename, rows in (("quality_events.csv", events), ("passage_black_origins.csv", passages)):
        with (output/filename).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
            writer.writeheader(); writer.writerows(rows)
    (output/"quality_scope.json").write_text(json.dumps(dict(path_cells=len(deadlines),
        event_rows=len(events), passage_black_cue_rows=len(passages),
        note="known非黒をsafeと略記。真の安全／誤判定ラベルではない。品質による選別は未実施。"),
        ensure_ascii=False, indent=2)+"\n")


if __name__ == "__main__":
    main()
