from types import SimpleNamespace

from cow_pkg.completion import all_cows_at_goal


def pose(x, y, z=0.0):
    return SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=z))


def test_all_expected_cows_required_and_boundary_inclusive():
    expected = ['cow0', 'cow1']
    assert all_cows_at_goal(expected, [pose(2, 0), pose(0, -2)], expected, (0, 0), 2)
    assert not all_cows_at_goal(['cow0'], [pose(0, 0)], expected, (0, 0), 2)
    assert not all_cows_at_goal(expected, [pose(2.001, 0), pose(0, 0)], expected, (0, 0), 2)
    assert not all_cows_at_goal([], [], [], (0, 0), 2)


def test_invalid_and_duplicate_positions_do_not_complete():
    expected = ['cow0', 'cow1']
    assert not all_cows_at_goal(['cow0', 'cow0'], [pose(0, 0)] * 2, expected, (0, 0), 2)
    assert not all_cows_at_goal(expected, [pose(float('nan'), 0), pose(0, 0)], expected, (0, 0), 2)
    assert not all_cows_at_goal(expected, [pose(0, 0)], expected, (0, 0), 2)
    assert all_cows_at_goal(['other', *expected], [pose(99, 99), pose(-8, -10), pose(-10, -10)],
                            expected, (-10, -10), 2)
