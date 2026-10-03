"""欠測分類が排他的で、共通セルmaskが統計に反映されることを確認する。"""
import csv
import json

from analyze_paired_terrain_support import main


def test_missing_support_categories_and_common_mask(tmp_path, monkeypatch):
    for name in ("independent", "temporal"):
        folder = tmp_path/name
        folder.mkdir()
        independent = name == "independent"
        hazard = [0., None, .1] if independent else [1., 1., 1.]
        terrain = [0., None, None] if independent else [0., 0., 0.]
        data = dict(x=[.05,.15,.25], y=[.05,.05,.05], hazard=hazard,
                    reference_hazard=hazard, slope_deg=terrain, roughness=terrain,
                    step_height=terrain, obstacle_height=([None]*3 if independent else [.1]*3),
                    pixel_count=[1,0,1], cell_min=[0.,None,0.], cell_max=[.01,None,.01])
        row = dict(frame_stamp_ns=1,passage_stamp_ns=2,cell_diagnostics_json=json.dumps(data))
        with (folder/"path_footprints.csv").open("w") as stream:
            writer=csv.DictWriter(stream,fieldnames=list(row)); writer.writeheader(); writer.writerow(row)
        (folder/"summary.json").write_text(json.dumps(dict(parameters=dict(hazard_obstacle_height_limit=.08))))
    monkeypatch.setattr("sys.argv", ["analysis",str(tmp_path/"independent"),str(tmp_path/"temporal"),
                                     "--output",str(tmp_path/"paired")])
    main()
    result=json.loads((tmp_path/"paired/summary.json").read_text())
    for key in ("independent_hazard_unknown", "independent_hazard_known_missing_terrain", "independent_all_terrain_valid"):
        assert result["totals"]["temporal_only_black_"+key] == 1
    common=result["common_masks"]["common_three_terrain"]
    assert common["cells"] == 1
    assert common["independent_black_cells"] == 0
    assert common["temporal_black_cells"] == 1
    assert result["totals"]["temporal_obstacle_black_current_range_below_limit"] == 2
