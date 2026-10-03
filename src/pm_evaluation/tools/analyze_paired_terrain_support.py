"""同じ撮像・通過stamp・odomセルで、hazard欠測と地形support不足を分離する。

独立側reference_hazard（単一画像obstacle許可）と時間融合側hazardを比較する。
率はfootprint内の延べセル評価数であり、uniqueな空間セル数ではない。
欠測をゼロへ置換せず、共通有効maskと全セルの比較を別々に報告する。
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def read_rows(folder):
    rows = list(csv.DictReader((folder / "path_footprints.csv").open()))
    keyed = {(int(r["frame_stamp_ns"]), int(r["passage_stamp_ns"])): r for r in rows}
    if len(keyed) != len(rows):
        raise ValueError("対応stampが重複しています")
    return keyed


def values(data, name):
    return np.asarray([np.nan if x is None else x for x in data[name]], dtype=float)


def black(v):
    """既存RViz/経路評価と同じ、丸め後100を黒とする。"""
    return np.isfinite(v) & (np.rint(np.clip(v, 0, 1)*100) >= 100)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("independent", type=Path)
    parser.add_argument("temporal", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    ind, tmp = read_rows(args.independent), read_rows(args.temporal)
    obstacle_limit=json.loads((args.temporal/"summary.json").read_text())["parameters"]["hazard_obstacle_height_limit"]
    if ind.keys() != tmp.keys():
        raise ValueError("同じframe/経路sampleではありません")
    totals = {}
    records = []
    modes = {name: dict(cells=0, independent_black_cells=0, temporal_black_cells=0,
                       footprints=0, independent_black_footprints=0, temporal_black_footprints=0,
                       complete_footprints=0, independent_black_complete=0, temporal_black_complete=0)
             for name in ("common_hazard", "common_plane", "common_three_terrain")}
    validity = {name: dict(independent=0, temporal=0, common=0)
                for name in ("hazard", "slope_deg", "roughness", "step_height", "all_three_terrain")}
    def add(name, n):
        totals[name] = totals.get(name, 0) + int(n)
    for key in sorted(ind):
        i = json.loads(ind[key]["cell_diagnostics_json"])
        t = json.loads(tmp[key]["cell_diagnostics_json"])
        # XYが一致しなければindex順を流用しない。今回は同じ入力pose・grid条件。
        if (len(i["x"]) != len(t["x"]) or not np.allclose(i["x"], t["x"], atol=1e-9, rtol=0)
                or not np.allclose(i["y"], t["y"], atol=1e-9, rtol=0)):
            raise ValueError("対応セルのodom XYが一致しません: " + str(key))
        ih, th = values(i, "reference_hazard"), values(t, "hazard")
        iv, tv = np.isfinite(ih), np.isfinite(th)
        ib, tb = black(ih), black(th)
        terrain_i = np.ones(len(ih), dtype=bool)
        terrain_t = np.ones(len(th), dtype=bool)
        plane_i, plane_t = np.isfinite(values(i,"slope_deg")), np.isfinite(values(t,"slope_deg"))
        for name in ("slope_deg", "roughness", "step_height"):
            a, b = np.isfinite(values(i,name)), np.isfinite(values(t,name))
            validity[name]["independent"] += int(a.sum())
            validity[name]["temporal"] += int(b.sum())
            validity[name]["common"] += int((a & b).sum())
            terrain_i &= a
            terrain_t &= b
        for name,a,b in (("hazard",iv,tv),("all_three_terrain",terrain_i,terrain_t)):
            validity[name]["independent"] += int(a.sum())
            validity[name]["temporal"] += int(b.sum())
            validity[name]["common"] += int((a & b).sum())
        add("footprints",1)
        add("cell_evaluations",len(ih))
        for prefix,v,b,geom,data in (("independent",iv,ib,terrain_i,i),("temporal",tv,tb,terrain_t,t)):
            add(prefix+"_known_footprints",v.any())
            add(prefix+"_black_footprints",b.any())
            add(prefix+"_fully_known_footprints",v.all())
            add(prefix+"_wholly_unknown_footprints",not v.any())
            add(prefix+"_black_or_unknown_footprints",b.any() or not v.all())
            add(prefix+"_observed_cells",(values(data,"pixel_count")>0).sum())
            add(prefix+"_observed_plane_unknown",((values(data,"pixel_count")>0)&~np.isfinite(values(data,"slope_deg"))).sum())
            add(prefix+"_known_missing_terrain",(v & ~geom).sum())
        masks = {"common_hazard":iv & tv,
                 "common_plane":iv & tv & plane_i & plane_t,
                 "common_three_terrain":iv & tv & terrain_i & terrain_t}
        row = dict(frame_stamp_ns=key[0], passage_stamp_ns=key[1], cells=len(ih))
        for name,mask in masks.items():
            stats=modes[name]
            stats["cells"]+=int(mask.sum())
            stats["independent_black_cells"]+=int((ib&mask).sum())
            stats["temporal_black_cells"]+=int((tb&mask).sum())
            if mask.any():
                stats["footprints"]+=1
                stats["independent_black_footprints"]+=int((ib&mask).any())
                stats["temporal_black_footprints"]+=int((tb&mask).any())
            if len(mask) and mask.all():
                stats["complete_footprints"]+=1
                stats["independent_black_complete"]+=int(ib.any())
                stats["temporal_black_complete"]+=int(tb.any())
            row[name+"_cells"]=int(mask.sum())
            row[name+"_independent_black_cells"]=int((ib&mask).sum())
            row[name+"_temporal_black_cells"]=int((tb&mask).sum())
        # 時間融合だけ黒となったセルを、独立側の欠測状態で排他的に分類する。
        newly_black=tb & ~ib
        classifications={"independent_hazard_unknown":newly_black&~iv,
                         "independent_hazard_known_missing_terrain":newly_black&iv&~terrain_i,
                         "independent_all_terrain_valid":newly_black&iv&terrain_i}
        for name,mask in classifications.items():
            add("temporal_only_black_"+name,mask.sum())
            row[name]=int(mask.sum())
        ob=black(values(t,"obstacle_height")/obstacle_limit)
        raw_range=values(i,"cell_max")-values(i,"cell_min")
        seen=np.isfinite(raw_range)
        add("temporal_obstacle_black_cells",ob.sum())
        add("temporal_obstacle_black_current_depth_seen",(ob&seen).sum())
        add("temporal_obstacle_black_current_range_below_limit",(ob&seen&~black(raw_range/obstacle_limit)).sum())
        add("temporal_obstacle_black_no_current_depth",(ob&~seen).sum())
        records.append(row)
    total=totals["cell_evaluations"]
    for d in validity.values():
        for name,n in list(d.items()):
            d[name+"_percent"]=100*n/total if total else None
    for d in modes.values():
        d["coverage_percent"]=100*d["cells"]/total if total else None
        for prefix in ("independent","temporal"):
            d[prefix+"_black_cell_percent"]=100*d[prefix+"_black_cells"]/d["cells"] if d["cells"] else None
            d[prefix+"_black_footprint_percent"]=100*d[prefix+"_black_footprints"]/d["footprints"] if d["footprints"] else None
    result=dict(totals=totals,validity=validity,common_masks=modes,
                independent_hazard="reference_hazard",temporal_hazard="hazard",
                note="共通maskのfootprint黒率はmask内の一部セルだけの値。coverageを併記し車体全域の安全率と解釈しない")
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/"summary.json").write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n")
    with (args.output/"paired_cells.csv").open("w") as stream:
        writer=csv.DictWriter(stream,fieldnames=list(records[0]))
        writer.writeheader();writer.writerows(records)
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
