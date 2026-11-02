"""モード毎の未知・危険の意味を保ち、argmaxでcueを消さないことを検証する。"""
import pytest
from pm_planner.perception_input import select_costs


def test_obstacle_only_ignores_terrain_and_full_keeps_independent_unknown():
    obstacle=[70,100,0,-1,-1]
    terrain=[100,0,-1,0,100]
    assert select_costs(obstacle,terrain,5,'obstacle_only').tolist()==obstacle
    assert select_costs(obstacle,terrain,5,'full_terrain').tolist()==[100,100,-1,-1,100]


def test_obstacle_first_phase_can_omit_terrain():
    assert select_costs([0,100,-1],[],3,'obstacle_only').tolist()==[0,100,-1]
    assert select_costs([0,100,-1],[],3,'full_terrain').tolist()==[-1,100,-1]


@pytest.mark.parametrize('a,b,mode',[([101],[],'obstacle_only'),([-2],[],'obstacle_only'),
                                    ([0],[0,0],'full_terrain'),([0],[],'invalid')])
def test_invalid_contract_is_not_interpreted_as_clear(a,b,mode):
    with pytest.raises(ValueError):select_costs(a,b,1,mode)
