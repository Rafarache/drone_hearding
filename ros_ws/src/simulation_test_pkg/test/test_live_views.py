from concurrent.futures import Future
from types import SimpleNamespace
from simulation_test_pkg.live_views import LiveViews
from simulation_test_pkg.models import Visualizations
from simulation_test_pkg.storage import atomic_json


def test_viewer_lifecycle_and_parameter_acknowledgments(tmp_path, manifest, monkeypatch):
    calls, spawned, events = [], [], []
    class Client:
        def service_is_ready(self): return True
        def call_async(self, request):
            calls.append(request.parameters)
            future = Future()
            future.set_result(SimpleNamespace(results=[SimpleNamespace(successful=True, reason='') for _ in request.parameters]))
            return future
    class Process:
        def __init__(self, command, **kwargs): self.command=command;self.returncode=None;self.pid=999999;spawned.append(self)
        def poll(self): return self.returncode
        def terminate(self): self.returncode=0
        def kill(self): self.returncode=-9
        def wait(self, **kwargs): return self.returncode
    monkeypatch.setattr('simulation_test_pkg.live_views.subprocess.Popen', Process)
    node=SimpleNamespace(create_client=lambda *a:Client(), destroy_client=lambda *a:None)
    recorder=SimpleNamespace(elapsed=2,writer=SimpleNamespace(event=lambda *a:events.append(a)))
    views=LiveViews(tmp_path,manifest,node,{'DISPLAY':':90'},recorder,virtual_display=True)
    views.tick();views.tick()
    assert views.applied and not spawned
    atomic_json(tmp_path/'live_request.json',{'revision':'on','settings':Visualizations(gazebo=True,rviz=True,occupancy=True).model_dump()})
    views.tick();views.tick()
    assert len(spawned)==2 and all(p.poll() is None for p in spawned)
    assert any(p.name=='show_occupancy_grid' and p.value.bool_value for parameters in calls for p in parameters)
    atomic_json(tmp_path/'live_request.json',{'revision':'off','settings':Visualizations().model_dump()})
    views.tick();views.tick()
    assert all(p.poll() is not None for p in spawned)
    assert len(events)==2
    views.close()
