"""local/globalの再生入力範囲と初期化完了時の検査を保証する。"""
from pathlib import Path

import pytest
from pm_evaluation.cli.bag_clock_player import MESSAGE_TYPES
from pm_evaluation.cli.replay_global_localization import INPUTS, OUTPUTS
from pm_evaluation.cli.plot_global_localization_replay import main


def test_replay_inputs_are_typed_and_non_actuating():
    assert set(INPUTS) <= set(MESSAGE_TYPES)
    assert {'/wit/mag','/wit/imu','/fix','/navpvt','/wheel/odometry'} <= set(INPUTS)
    assert not {'/cmd_vel','/cmd_vel_joy','/pm/joy','/motor_state'} & set(INPUTS)
    assert {'/odometry/local','/odometry/global',
            '/localization/initialization/status'} <= set(OUTPUTS)


def test_plot_rejects_uninitialized_bag(monkeypatch, tmp_path):
    """READYなしの結果を、見た目だけの初期整列済み軌跡として表示しない。"""
    module='pm_evaluation.cli.plot_global_localization_replay'
    monkeypatch.setattr(module+'.read_replay', lambda _: {
        '/localization/initialization/status':[{'state':'WAITING'}]})
    monkeypatch.setattr('sys.argv',['plot_global_localization_replay',str(tmp_path),
                                   '--basemap',str(Path('unused.npz'))])
    with pytest.raises(SystemExit,match='READY'):
        main()
