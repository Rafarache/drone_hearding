import json
from pathlib import Path
import sys
import time
import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError
from simulation_test_pkg.models import BatchSpec, Scenario
from simulation_test_pkg.server import create_app
from simulation_test_pkg.storage import Store, atomic_json
from simulation_test_pkg.supervisor import Supervisor, submit_batch


def fake_snapshot(directory, scenario):
    directory.mkdir(parents=True)
    (directory / 'config').mkdir()
    result = {'scenario': scenario.model_dump(), 'entities': []}
    atomic_json(directory / 'manifest.json', result)
    return result


def wait_until(predicate):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.05)
    raise AssertionError('Queue did not reach expected state')


def test_five_repetitions_snapshots_order_and_single_owner(tmp_path, scenario):
    store = Store(tmp_path)
    saved = store.save_scenario(scenario.model_dump())
    batch = submit_batch(store, {'items': [{'scenario_id': saved['id'], 'repeats': 5}]}, fake_snapshot)
    changed = scenario.model_copy(update={'name': 'Edited after queueing'})
    store.save_scenario(changed.model_dump(), saved['id'])
    assert all(r['scenario']['name'] == 'Fixture' for r in store.runs())
    # A controlled subprocess is enough to verify the supervisor's actual
    # five-run queue, process lifecycle, and independent result files.
    worker = tmp_path / 'fake_worker.py'
    worker.write_text('''import argparse,json,time,pathlib
p=argparse.ArgumentParser();p.add_argument('--run-dir');a=p.parse_args();d=pathlib.Path(a.run_dir)
assert not (d.parent.parent/'busy').exists()
(d.parent.parent/'busy').write_text('busy')
time.sleep(.1)
(d/'state.json').write_text(json.dumps({'status':'success','simulation_duration_s':1}))
(d.parent.parent/'busy').unlink()
''')
    supervisor = Supervisor(store, [sys.executable, str(worker)])
    supervisor.start()
    try:
        with pytest.raises(RuntimeError, match='already owns'):
            Supervisor(store).start()
        wait_until(lambda: all(r['status'] == 'success' for r in store.runs()))
    finally:
        supervisor.stop()
    assert [r['repetition'] for r in store.runs()] == [1, 2, 3, 4, 5]
    assert len({r['id'] for r in store.runs()}) == 5
    assert store.batches()[0]['id'] == batch


def test_cancel_queued_batch_and_reconcile_interrupted_run(tmp_path, scenario):
    store = Store(tmp_path)
    batch = submit_batch(store, {'items': [{'scenario': scenario.model_dump(), 'repeats': 5}]}, fake_snapshot)
    first = store.claim()
    store.cancel(batch_id=batch)
    assert sum(r['status'] == 'cancelled' for r in store.runs()) == 4
    supervisor = Supervisor(store)
    supervisor.start()
    supervisor.stop()
    assert store.run(first['id'])['status'] == 'interrupted'


@pytest.mark.parametrize('change', [{'duration_s': 1201}, {'duration_s': 0}, {'drones': []},
                                    {'goal': {'x': 999, 'y': 0}},
                                    {'speeds': {'cow_linear_mps': float('nan')}}])
def test_invalid_experiments_rejected(scenario, change):
    with pytest.raises(ValidationError):
        Scenario.model_validate(dict(scenario.model_dump(), **change))


def test_api_scenario_validation_missing_runs_and_same_origin(tmp_path, scenario):
    app = create_app(tmp_path, manage_worker=False, default_factory=lambda: scenario)
    with TestClient(app) as client:
        assert client.get('/api/health').status_code == 200
        result = client.post('/api/scenarios', json=scenario.model_dump())
        assert result.status_code == 201
        saved = result.json()
        assert client.get('/api/scenarios').json()[0]['id'] == saved['id']
        assert client.post('/api/scenarios', json=dict(scenario.model_dump(), duration_s=1201)).status_code == 422
        assert client.post('/api/scenarios', json=scenario.model_dump(), headers={'origin': 'https://other.example'}).status_code == 403
        assert client.get('/api/runs/missing').status_code == 404
        assert client.get('/api/runs/../../etc/passwd/trajectory').status_code == 404
        assert client.delete('/api/scenarios/'+saved['id']).status_code == 204
        assert client.get('/api/scenarios').json() == []


def test_reconcile_terminal_state_not_yet_committed_to_catalog(tmp_path, scenario):
    store = Store(tmp_path)
    submit_batch(store, {'items': [{'scenario': scenario.model_dump()}]}, fake_snapshot)
    run = store.claim()
    atomic_json(store.run_dir(run['id']) / 'state.json', {'status': 'success', 'simulation_duration_s': 3})
    supervisor = Supervisor(store)
    supervisor.start()
    supervisor.stop()
    with __import__('simulation_test_pkg.storage', fromlist=['database']).database(store.path) as db:
        row = db.execute('SELECT status FROM runs WHERE id=?', (run['id'],)).fetchone()
        assert row['status'] == 'success'


def test_metrics_csv_includes_fields_specific_to_cows(tmp_path, scenario):
    app = create_app(tmp_path, manage_worker=False, default_factory=lambda: scenario)
    store = app.state.store
    submit_batch(store, {'items': [{'scenario': scenario.model_dump()}]}, fake_snapshot)
    run = store.runs()[0]
    store.update(run['id'], {'status': 'success', 'summary': {'entities': [
        {'name': 'drone0', 'kind': 'drone', 'distance_xy_m': 2},
        {'name': 'cow0', 'kind': 'cow', 'distance_xy_m': 3, 'first_goal_arrival_s': 4,
         'final_inside_goal': True}]}})
    with TestClient(app) as client:
        result = client.get('/api/runs/'+run['id']+'/export/metrics')
        assert result.status_code == 200
        assert 'first_goal_arrival_s' in result.text
        assert 'final_inside_goal' in result.text
        assert 'cow0,cow,3,4,True' in result.text
