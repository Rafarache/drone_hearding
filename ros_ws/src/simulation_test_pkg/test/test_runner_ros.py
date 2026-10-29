"""Opt-in real Gazebo/YOLO smoke tests; each owns its isolated simulator."""
import os
import time
import pytest
from simulation_test_pkg.configuration import defaults
from simulation_test_pkg.search import history
from simulation_test_pkg.storage import Store, TERMINAL, trajectory
from simulation_test_pkg.supervisor import Supervisor, submit_batch

pytestmark = pytest.mark.skipif(os.environ.get('SIM_TEST_RUN_REAL') != '1',
                                reason='Set SIM_TEST_RUN_REAL=1 to start isolated Gazebo missions')


def test_actual_launch_early_success_then_recorded_timeout(tmp_path):
    base = defaults().model_dump()
    early = dict(base, name='Smoke: early success', drones=[{'x': 0, 'y': 0}],
                 cows=[{'x': -10, 'y': -10, 'z': .01}], duration_s=20)
    moving = dict(base, name='Smoke: recorded movement', drones=[{'x': 0, 'y': 0}, {'x': 0, 'y': 3}],
                  cows=[{'x': 6, 'y': 0, 'z': .01}], duration_s=20)
    store = Store(tmp_path)
    batch = submit_batch(store, {'items': [{'scenario': early}, {'scenario': moving}]})
    supervisor = Supervisor(store)
    supervisor.start()
    try:
        deadline = time.monotonic() + 180
        while time.monotonic() < deadline:
            runs = [r for r in store.runs() if r['batch_id'] == batch]
            if all(r['status'] in TERMINAL for r in runs):
                break
            time.sleep(.25)
        assert [r['status'] for r in runs] == ['success', 'timed_out'], runs
        assert runs[0]['simulation_duration_s'] == 0
        assert runs[1]['simulation_duration_s'] == 20
        data = trajectory(store.run_dir(runs[1]['id']) / 'trajectory.sqlite3')
        assert len(data['frames']) >= 100
        assert data['detections'], 'YOLO must process images even without positive detections'
        assert runs[1]['summary']['distance_drones_m'] > .01
        search = history(store.run_dir(runs[1]['id']) / 'trajectory.sqlite3')
        assert search['available'] and len(search['drones']) == 2
        assert all(drone['cells'] for drone in search['drones'])
        assert all(any(state['goal'] is not None for state in drone['goals']) for drone in search['drones'])
    finally:
        supervisor.stop()
