from concurrent.futures import Future
from types import SimpleNamespace
from simulation_test_pkg.live_views import LiveViews, gazebo_viewer_environment
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
        def __init__(self, command, **kwargs): self.command=command;self.environment=kwargs["env"];self.returncode=None;self.pid=999999;spawned.append(self)
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
    gazebo = next(p for p in spawned if p.command[0] == 'gzclient')
    assert gazebo.environment['GAZEBO_MODEL_DATABASE_URI'].startswith('file://')
    assert '--verbose' in gazebo.command
    rviz = next(p for p in spawned if p.command[0] == 'rviz2')
    assert 'GAZEBO_MODEL_DATABASE_URI' not in rviz.environment
    assert any(p.name=='show_occupancy_grid' and p.value.bool_value for parameters in calls for p in parameters)
    atomic_json(tmp_path/'live_request.json',{'revision':'off','settings':Visualizations().model_dump()})
    views.tick();views.tick()
    assert all(p.poll() is not None for p in spawned)
    assert len(events)==2
    views.close()


def test_gazebo_catalog_is_local_and_model_paths_are_preserved(tmp_path, monkeypatch):
    import xml.etree.ElementTree as ET
    from urllib.parse import urlparse, unquote
    monkeypatch.setattr('ament_index_python.packages.get_package_share_directory',
                        lambda package: '/installed/' + package)
    original = {'DISPLAY': ':0', 'GAZEBO_MASTER_URI': 'http://127.0.0.1:45678',
                'GAZEBO_MODEL_PATH': '/custom/models:/installed/cow_pkg/models'}
    environment = gazebo_viewer_environment(tmp_path, original)
    catalog_uri = urlparse(environment['GAZEBO_MODEL_DATABASE_URI'])
    assert catalog_uri.scheme == 'file'
    catalog = ET.parse(unquote(catalog_uri.path) + '/database.config').getroot()
    assert catalog.tag == 'database' and list(catalog.find('models')) == []
    paths = environment['GAZEBO_MODEL_PATH'].split(':')
    assert '/custom/models' in paths and len(paths) == len(set(paths))
    assert all('/installed/' + p + '/models' in paths
               for p in ('sjtu_drone_description', 'turtlebot3_gazebo', 'cow_pkg'))
    assert environment['GAZEBO_MASTER_URI'] == original['GAZEBO_MASTER_URI']
    assert 'GAZEBO_MODEL_DATABASE_URI' not in original
