"""人工説明例の分類・確認・解除と、HTMLの自己完結性を検証する。"""
import importlib.util
import json
from pathlib import Path

import numpy as np

from pm_evaluation.height_candidates import CandidateOptions
from pm_evaluation.height_demo_html import write_demo_html


def load_demo():
    path = Path(__file__).resolve().parents[1]/'tools/demo_height_candidate_scene.py'
    spec = importlib.util.spec_from_file_location('height_demo', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def test_synthetic_geometry_and_clearance():
    demo = load_demo()
    scene = demo.SceneBuilder(.1,[20.,.03,.07,.08],CandidateOptions(),4.,100)
    snapshots = []
    for index,points,pixels,pose,baseline,planes,usable in demo.synthetic_frames():
        objects,clears,valid = scene.update(points,pixels,pose,pose[0],index,baseline,planes,usable)
        snapshots.append(scene.snapshot(points,pose,pose[0],index,index*.1,objects,clears,valid))
    kinds = {o['kind'] for o in snapshots[0]['objects']}
    assert {'ground_provisional','broad','thin','sparse'} <= kinds
    # 消える高点セルはどのNでも解除し、持続する高点は確認済みのまま。
    for n in (1,3,5):
        assert not scene.stores[n][(10,-6)].active
        assert scene.stores[n][(8,-5)].confirmed
        assert any(f['clears'][str(n)] > 0 for f in snapshots)
    assert len(snapshots[-1]['points']) <= 100
    assert max(o['bounds'][5]-o['bounds'][4] for o in snapshots[0]['objects'] if o['kind']=='broad') > 1.
    # NumPy整数やNaNがHTML埋込データへ漏れていないことも検査する。
    json.dumps(snapshots, allow_nan=False)


def test_html_is_self_contained_and_escapes_script(tmp_path):
    target = tmp_path/'demo.html'
    write_demo_html(target,{'metadata':{'source':'</script>'},'frames':[]})
    html = target.read_text()
    assert '<script src=' not in html and '__SCENE_JSON__' not in html
    assert '\\u003c/script>' in html


def test_residuals_respect_plane_and_unknown():
    demo = load_demo()
    points = np.array([[.03,.04,.2],[.05,.06,.3],[.15,.05,1.]])
    planes = np.zeros((1,2,3)); planes[:,:,2] = .1
    result = demo.maximum_residuals(points,np.array([0.,0.]),(1,2),.1,planes,np.array([[True,False]]))
    assert np.isclose(result[0],.2) and np.isnan(result[1])
