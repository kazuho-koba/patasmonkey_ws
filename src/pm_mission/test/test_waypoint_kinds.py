"""必須到達属性の保存・ROS往復・Qt編集を検証する。"""
import copy
from pathlib import Path
from types import SimpleNamespace
import pytest
from pm_mission.model import load_document,save_document,validate,to_message,from_message,ATTRIBUTES,LABELS


def test_visit_yaml_and_ros_roundtrip(tmp_path):
    mission=load_document(Path(__file__).resolve().parents[1]/'missions/home.yaml')
    mission['waypoints'][1]['attribute']='visit'
    save_document(tmp_path/'mission.yaml',mission)
    saved=load_document(tmp_path/'mission.yaml')
    assert from_message(to_message(saved))==saved
    assert ATTRIBUTES.index('confirm')==4 and ATTRIBUTES.index('visit')==5
    assert to_message(saved).waypoints[1].attribute==5
    broken=copy.deepcopy(saved);broken['waypoints'][-1]['attribute']='visit'
    with pytest.raises(ValueError):validate(broken,True)


def test_qt_editor_can_select_required_visit(tmp_path):
    from PyQt5.QtWidgets import QApplication
    import yaml
    from pm_mission.app import MissionWindow
    from pm_mission.backend import Events
    app=QApplication.instance() or QApplication([])
    config=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/mission.yaml').read_text())
    config['map'].update(online_tiles_enabled=False,tile_cache_dir=str(tmp_path/'tiles'))
    # 実機・ROS serviceへ接続せず属性comboのdocument更新だけを検証する。
    window=MissionWindow(config,SimpleNamespace(client=SimpleNamespace(service_is_ready=lambda:False)),Events());window.timer.stop()
    window.document=load_document(Path(__file__).resolve().parents[1]/'missions/home.yaml');window.refresh()
    combo=window.table.cellWidget(1,3)
    assert '必ず寄る点' in [combo.itemText(i) for i in range(combo.count())]
    combo.setCurrentIndex(ATTRIBUTES.index('visit'))
    assert window.document['waypoints'][1]['attribute']=='visit'
    assert '必須' in LABELS[ATTRIBUTES.index('goal')]
    window.dirty=False;window.close();app.processEvents()
