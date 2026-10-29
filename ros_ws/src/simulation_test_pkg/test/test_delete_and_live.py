import json
import pytest
from fastapi.testclient import TestClient
from simulation_test_pkg.models import Visualizations
from simulation_test_pkg.server import create_app
from simulation_test_pkg.storage import Store, atomic_json
from simulation_test_pkg.supervisor import submit_batch
from test_queue_api import fake_snapshot


def test_single_bulk_delete_and_preserved_scenarios(tmp_path, scenario):
    app = create_app(tmp_path, manage_worker=False, default_factory=lambda: scenario)
    store = app.state.store
    saved = store.save_scenario(scenario.model_dump())
    submit_batch(store, {'items': [{'scenario_id': saved['id'], 'repeats': 3}]}, fake_snapshot)
    runs = store.runs()
    for run in runs:
        store.update(run['id'], {'status': 'success'})
    with TestClient(app) as client:
        assert client.delete('/api/runs/'+runs[0]['id']).status_code == 200
        assert not store.run_dir(runs[0]['id']).exists()
        assert client.post('/api/runs/delete', json={'run_ids': [r['id'] for r in runs[1:]]}).status_code == 200
        assert store.runs() == [] and store.batches() == []
        assert all(not store.run_dir(r['id']).exists() for r in runs)
        assert store.scenario(saved['id'])


def test_delete_is_atomic_and_rejects_active_runs(tmp_path, scenario):
    store = Store(tmp_path)
    submit_batch(store, {'items': [{'scenario': scenario.model_dump(), 'repeats': 2}]}, fake_snapshot)
    active = store.claim()
    queued = next(r for r in store.runs() if r['id'] != active['id'])
    with pytest.raises(ValueError, match='Cancel or finish'):
        store.delete_runs([queued['id'], active['id']])
    assert len(store.runs()) == 2 and store.run_dir(queued['id']).exists()
    with pytest.raises(KeyError):
        store.delete_runs([queued['id'], 'a'*32])
    assert len(store.runs()) == 2
    store.delete_runs([queued['id']])
    assert store.claim() is None and store.run(active['id'])


def test_interrupted_delete_restores_referenced_data(tmp_path, scenario):
    store = Store(tmp_path)
    submit_batch(store, {'items': [{'scenario': scenario.model_dump()}]}, fake_snapshot)
    run = store.runs()[0]
    trash = tmp_path / '.deleting';trash.mkdir()
    store.run_dir(run['id']).replace(trash / (run['id']+'-transaction'))
    Store(tmp_path)
    assert store.run_dir(run['id']).exists() and list(trash.iterdir()) == []


def test_live_request_and_active_delete_validation(tmp_path, scenario):
    app = create_app(tmp_path, manage_worker=False, default_factory=lambda: scenario)
    store = app.state.store
    submit_batch(store, {'items': [{'scenario': scenario.model_dump()}]}, fake_snapshot)
    run = store.claim();path = store.run_dir(run['id'])/'manifest.json'
    manifest = json.loads(path.read_text());manifest['live_visualization_controls'] = True
    atomic_json(path, manifest)
    with TestClient(app) as client:
        settings = Visualizations(gazebo=True, occupancy=True).model_dump()
        response = client.post('/api/runs/'+run['id']+'/visualizations', json=settings)
        assert response.status_code == 202 and response.json()['settings']['gazebo']
        assert not store.run(run['id'])['scenario']['visualization']['gazebo']
        assert client.delete('/api/runs/'+run['id']).status_code == 409
        assert client.post('/api/runs/'+run['id']+'/visualizations', json={'unexpected':True}).status_code == 422
        store.update(run['id'], {'status':'success'})
        assert client.post('/api/runs/'+run['id']+'/visualizations', json=settings).status_code == 409
