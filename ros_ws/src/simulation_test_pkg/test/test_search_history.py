import sqlite3
from simulation_test_pkg.search import history
from simulation_test_pkg.storage import TrajectoryWriter
from simulation_test_pkg.telemetry import Recording
from test_recording import entities


def state(stamp, cells, goal=None):
    return {'version': 1, 'frame_id': 'map', 'stamp_s': stamp,
            'grid': {'width': 4, 'height': 3, 'resolution': .5, 'origin': [-1, -1], 'seen_cells': cells},
            'goal': goal}


def goal(x):
    return {'viewpoint': [x, 0], 'patch_center': [x+2, 0], 'strength': .6,
            'patch': [[x+1, -1], [x+3, -1], [x+3, 1], [x+1, 1]]}


def test_time_bounded_cells_and_goal_changes(tmp_path, manifest):
    path = tmp_path / 'positions.sqlite3'
    writer = TrajectoryWriter(path)
    manifest['entities'].append({'name': '/fixture_drone1', 'kind': 'drone', 'index': 1})
    recorder = Recording(manifest, writer)
    recorder.update_search('/fixture_drone0', state(99, [0]))
    recorder.update_clock(100)
    recorder.update_positions([*entities(), dict(entities()[0], name='/fixture_drone1')])
    recorder.update_clock(101)
    recorder.update_search('/fixture_drone0', state(101, [0, 3], goal(2)))
    recorder.update_search('/fixture_drone1', state(101, [5], goal(-2)))
    recorder.update_clock(102)
    recorder.update_search('/fixture_drone0', state(102, [0, 1, 3], goal(4)))
    recorder.update_clock(103)
    recorder.update_search('/fixture_drone0', state(103, [0, 1, 3]))
    writer.close()
    first, second = history(path, 1.5)['drones']
    assert first['cells'] == [[0, 0], [3, 1]] and second['cells'] == [[5, 1]]
    assert first['goals'][-1]['goal']['viewpoint'] == [2., 0.]
    latest = history(path)['drones'][0]
    assert latest['cells'] == [[0, 0], [3, 1], [1, 2]]
    assert latest['goals'][-1] == {'t': 3, 'goal': None}
    assert history(path, .5)['drones'][0]['cells'] == [[0, 0]]


def test_old_recordings_without_search_are_not_modified(tmp_path):
    path = tmp_path / 'legacy.sqlite3'
    with sqlite3.connect(path) as db:
        db.execute('CREATE TABLE positions(t REAL)')
    assert history(path) == {'available': False, 'drones': []}
    with sqlite3.connect(path) as db:
        assert db.execute("SELECT name FROM sqlite_master WHERE type='table'").fetchall() == [('positions',)]


def test_rejected_optional_states_preserve_permanent_memory(tmp_path, manifest):
    path = tmp_path / 'positions.sqlite3'
    writer = TrajectoryWriter(path)
    recorder = Recording(manifest, writer)
    recorder.update_clock(100)
    recorder.update_positions(entities())
    recorder.update_search('/fixture_drone0', state(100, [1]))
    for invalid in ('[]', state(101, [-1]), state(101, []), state(float('nan'), [1])):
        recorder.update_search('/fixture_drone0', invalid)
    writer.close()
    assert not recorder.outcome
    assert history(path)['drones'][0]['cells'] == [[1, 0]]
    with sqlite3.connect(path) as db:
        assert db.execute("SELECT count(*) FROM events WHERE type='search_rejected'").fetchone()[0] == 4
