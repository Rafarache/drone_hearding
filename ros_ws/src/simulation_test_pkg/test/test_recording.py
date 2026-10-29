import math
import pytest
from simulation_test_pkg.metrics import summarize
from simulation_test_pkg.storage import TrajectoryWriter, trajectory
from simulation_test_pkg.telemetry import Recording


def entities(x=5, drone_x=-5, z=1):
    return [{'name': '/fixture_drone0', 'kind': 'drone', 'pose': [drone_x, 0, z, 0, 0, 0, 1], 'velocity': [0, 0, 0]},
            {'name': 'cow0', 'kind': 'cow', 'pose': [x, 0, 0, 0, 0, 0, 1], 'velocity': [1, 0, 0]}]


def test_readiness_limit_and_no_wall_clock_timeout(tmp_path, manifest):
    path = tmp_path / 'trajectory.sqlite3'
    writer = TrajectoryWriter(path)
    wall = [0.0]
    recorder = Recording(manifest, writer, lambda: wall[0])
    recorder.update_positions(entities())
    assert recorder.origin is None
    recorder.update_clock(200)
    recorder.update_positions(entities()[:1])
    assert recorder.origin is None
    recorder.update_positions(entities())
    assert recorder.origin == 200
    # Real time can exceed 20 minutes; progressing clocks determine timeout.
    for t in range(1, 1200):
        wall[0] += 3
        recorder.update_clock(200 + t)
        recorder.update_positions(entities())
        recorder.check_watchdogs()
    assert not recorder.outcome
    recorder.update_clock(1400)
    assert recorder.outcome == 'timed_out' and recorder.elapsed == 1200
    recorder.update_positions(entities(x=999))
    writer.close()
    assert all(f['t'] <= 1200 for f in trajectory(path)['frames'])


def test_success_requires_all_cows_and_clears_at_one_snapshot(tmp_path, manifest):
    manifest['scenario']['cows'].append({'x': 8, 'y': 0, 'z': .01})
    manifest['entities'].append({'name': 'cow1', 'kind': 'cow', 'index': 1})
    manifest['scenario']['goal'] = {'x': 0, 'y': 0, 'radius_m': 2}
    writer = TrajectoryWriter(tmp_path / 'positions.sqlite3')
    recorder = Recording(manifest, writer)
    recorder.update_clock(0)
    data = entities(x=0)
    recorder.update_positions(data)
    assert recorder.origin is None
    other = dict(data[-1], name='cow1', pose=[8, 0, 0, 0, 0, 0, 1])
    recorder.update_positions([*data, other])
    assert recorder.outcome is None
    recorder.update_clock(.1)
    recorder.update_positions([*data, dict(other, pose=[2, 0, 0, 0, 0, 0, 1])])
    assert recorder.outcome == 'success'
    writer.close()


def test_metrics_stops_3d_and_gaps(tmp_path, manifest):
    path = tmp_path / 'positions.sqlite3'
    writer = TrajectoryWriter(path)
    writer.frame(0, entities(x=0, drone_x=0, z=0))
    writer.frame(1, entities(x=1, drone_x=3, z=4))
    writer.frame(2, entities(x=1, drone_x=3, z=4))
    writer.frame(5, entities(x=100, drone_x=100, z=100))
    writer.close()
    result = summarize(path, manifest['scenario'], ['/fixture_drone0', 'cow0', 'missing'])
    drone, cow = result['entities']
    assert drone['distance_xy_m'] == 3 and drone['distance_3d_m'] == 5
    assert drone['average_speed_xy_mps'] == 1.5
    assert cow['distance_xy_m'] == 1 and cow['average_speed_xy_mps'] == .5
    assert drone['incomplete'] and drone['missing_intervals'] == [[2, 5]]
    assert result['missing_entities'] == ['missing']


@pytest.mark.parametrize('failure', ['startup', 'clock', 'models', 'reset', 'cancel'])
def test_failures_and_cancellation(tmp_path, manifest, failure):
    now = [0]
    writer = TrajectoryWriter(tmp_path / 'positions.sqlite3')
    recorder = Recording(manifest, writer, lambda: now[0])
    if failure != 'startup':
        recorder.update_clock(100)
        recorder.update_positions(entities())
    if failure == 'reset':
        recorder.update_clock(99)
    elif failure == 'cancel':
        recorder.finish('cancelled', 'Operator cancelled')
    else:
        now[0] = 121 if failure == 'startup' else 61
        if failure == 'models':
            recorder.update_clock(101)
        recorder.check_watchdogs()
    assert recorder.outcome == ('cancelled' if failure == 'cancel' else 'failed')
    writer.close()


def test_detection_layer_does_not_change_truth_or_metrics(tmp_path, manifest):
    path = tmp_path / 'positions.sqlite3'
    writer = TrajectoryWriter(path)
    recorder = Recording(manifest, writer)
    recorder.update_detections([(99, 99)])
    recorder.update_clock(10)
    recorder.update_positions(entities())
    recorder.update_detections([(100, 100), (math.nan, 0)])
    recorder.update_clock(10.5)
    recorder.update_positions(entities())
    recorder.update_detections([])
    writer.close()
    data = trajectory(path, .4, 1)
    assert data['detections'] == [{'t': 0, 'points': [[100, 100]]}, {'t': .5, 'points': []}]
    assert summarize(path, manifest['scenario'], ['cow0'])['entities'][0]['distance_xy_m'] == 0


def test_sampling_uses_time_buckets_without_accumulating_cadence_drift(tmp_path, manifest):
    path = tmp_path / 'positions.sqlite3'
    writer = TrajectoryWriter(path)
    recorder = Recording(manifest, writer)
    recorder.update_clock(0)
    recorder.update_positions(entities())
    for step in range(1, 101):
        recorder.update_clock(step * .01)
        recorder.update_positions(entities())
    writer.close()
    assert len(trajectory(path)['frames']) == 11
