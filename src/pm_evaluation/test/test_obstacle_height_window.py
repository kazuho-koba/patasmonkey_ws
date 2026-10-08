"""解除窓の境界、安全側保持、未観測中立を確認する。"""
import pytest
from pm_evaluation.obstacle_height_window import ObstacleHeightWindow


@pytest.mark.parametrize('n', [1, 3, 5])
@pytest.mark.parametrize('confirmed', [False, True])
def test_clear_and_reappearance(n, confirmed):
    state = ObstacleHeightWindow(n, .08)
    state.update(.16, 0, confirmed)
    assert state.active and state.value >= .08
    # unknownや未観測は窓を進めず、黒も消さない。
    state.update(float('nan'), 1)
    assert len(state.history) == 1 and state.active
    for stamp in range(2, n+3):
        previous = state.update(0., stamp)
        if previous:
            assert previous == ('confirmed' if confirmed or n == 1 else 'pending')
            break
    assert not state.active and state.value < .0796
    state.update(.09, 99)
    assert state.active and len(state.history) == 1


def test_rounding_boundary_and_new_hit():
    state = ObstacleHeightWindow(3, .08)
    for stamp in range(3):
        state.update(0., stamp)
    state.update(.24, 3)
    assert state.active and len(state.history) == 1
    state.update(0., 4); state.update(0., 5)
    assert state.active  # 平均0.08はまだ黒。
    assert state.update(0., 6) == 'pending'


@pytest.mark.parametrize('n', [3, 5])
def test_full_window_required_even_when_mean_is_low(n):
    state = ObstacleHeightWindow(n, .08)
    state.update(.081, 0)
    for stamp in range(1, n-1):
        assert state.update(0., stamp) is None
        assert state.active
    assert state.update(0., n-1) == 'pending'


def test_confirmation_is_held_until_clear():
    state = ObstacleHeightWindow(3, .08)
    state.update(.1, 0)
    state.update(.1, 1)
    state.update(.1, 2, confirmation=True)
    assert state.confirmed
    assert state.update(float('nan'), 3) is None
    assert state.confirmed
    assert state.update(.01, 4) == 'confirmed'
