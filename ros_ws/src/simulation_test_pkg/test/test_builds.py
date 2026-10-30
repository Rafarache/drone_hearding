import threading
from fastapi.testclient import TestClient
import pytest
from simulation_test_pkg.builds import BuildConflict, BuildControl
from simulation_test_pkg.server import create_app
from simulation_test_pkg.storage import Store, atomic_json
from simulation_test_pkg.supervisor import submit_batch
from test_queue_api import fake_snapshot


def test_missing_controller_and_persistent_failed_build(tmp_path):
    control = BuildControl(Store(tmp_path))
    assert control.status()['available'] is False
    with pytest.raises(BuildConflict, match='unavailable'):
        control.start()
    atomic_json(tmp_path / 'build-state.json', {'status': 'failed'})
    with pytest.raises(BuildConflict, match='successful build'):
        with control.submission():
            pytest.fail('A failed build must not admit experiments')


@pytest.mark.parametrize('status', ['queued', 'starting', 'running'])
def test_build_rejects_pending_experiments(tmp_path, scenario, monkeypatch, status):
    store = Store(tmp_path)
    submit_batch(store, {'items': [{'scenario': scenario.model_dump()}]}, fake_snapshot)
    store.update(store.runs()[0]['id'], {'status': status})
    control = BuildControl(store)
    monkeypatch.setattr(control, '_request', lambda _: pytest.fail('Must check experiments before building'))
    with pytest.raises(BuildConflict, match='queued experiments'):
        control.start()


def test_build_api_blocks_batch_and_rerun_during_build(tmp_path, scenario, monkeypatch):
    app = create_app(tmp_path, manage_worker=False)
    store, control = app.state.store, app.state.builds
    state = {'status': 'idle', 'available': True}
    def request(action):
        if action == 'build':
            state.update(status='building', id='test-build')
        return dict(state)
    monkeypatch.setattr(control, '_request', request)
    with TestClient(app) as client:
        assert client.post('/api/build', headers={'origin': 'https://elsewhere.test'}).status_code == 403
        assert client.post('/api/build').status_code == 202
        assert client.get('/api/build').json()['status'] == 'building'
        response = client.post('/api/batches', json={'items': [{'scenario': scenario.model_dump()}]})
        assert response.status_code == 409
        assert store.runs() == []
        # Reruns use frozen XML, but must pass through the same gate.
        from test_configuration import sources, SOURCE
        from simulation_test_pkg.configuration import snapshot
        submit_batch(store, {'items': [{'scenario': scenario.model_dump()}]},
                     lambda d, s: snapshot(d, s, sources(), lambda *_: SOURCE / 'sjtu_drone_description/worlds/farm_no_animals.world'))
        previous = store.runs()[0]
        store.update(previous['id'], {'status': 'success'})
        assert client.post('/api/runs/' + previous['id'] + '/rerun').status_code == 409
        state['status'] = 'failed'
        assert client.post('/api/batches', json={'items': [{'scenario': scenario.model_dump()}]}).status_code == 409
        state['status'] = 'succeeded'
        with control.submission():
            pass


def test_build_and_enqueue_share_the_same_gate(tmp_path, monkeypatch):
    control = BuildControl(Store(tmp_path))
    state = {'status': 'idle'}
    monkeypatch.setattr(control, '_request', lambda _: state)
    entered, release, requested = threading.Event(), threading.Event(), threading.Event()
    def submission():
        with control.submission():
            entered.set()
            release.wait(3)
    def build():
        control.start()
        requested.set()
    first = threading.Thread(target=submission)
    first.start();assert entered.wait(2)
    second = threading.Thread(target=build)
    second.start()
    try:
        assert not requested.wait(.1)
    finally:
        release.set();first.join(3);second.join(3)
    assert requested.is_set()
