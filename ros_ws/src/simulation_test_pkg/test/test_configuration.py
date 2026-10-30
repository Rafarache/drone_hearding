import json
from pathlib import Path
import xml.etree.ElementTree as ET
from simulation_test_pkg.configuration import FILES, defaults, snapshot
from simulation_test_pkg.models import Scenario

SOURCE = Path(__file__).parents[2]


def sources():
    return {name: SOURCE / package / 'config' / filename for name, (package, filename) in FILES.items()}


def test_frozen_inputs_override_speeds_and_preserve_defaults(tmp_path):
    original = {name: path.read_bytes() for name, path in sources().items()}
    scenario = defaults(sources()).model_copy(update={'name': 'Speed experiment'})
    scenario.speeds.cow_linear_mps = .25
    scenario.speeds.drone_linear_mps = .7
    scenario.goal.radius_m = 9
    scenario.goal.completion_radius_m = 11
    scenario.goal.extended_radius_m = 12
    manifest = snapshot(tmp_path / 'run', scenario, sources(),
                        lambda *_: SOURCE / 'sjtu_drone_description/worlds/farm_no_animals.world')
    directory = tmp_path / 'run/config'
    assert ET.parse(directory / 'cow_config.xml').getroot().find('motion').get('linear_velocity_mps') == '0.25'
    assert ET.parse(directory / 'drone_control_config.xml').getroot().find('motion').get('max_linear_velocity_mps') == '0.7'
    assert float(ET.parse(directory / 'grid_config.xml').getroot().find('goal_quiet_radius_m').text) == 12
    root = ET.parse(directory / 'simulation.xml').getroot()
    assert float(root.find('herding').get('goal_radius_m')) == 9
    assert float(root.find('herding').get('final_goal_reached_radius_m')) == 11
    assert root.find('runtime').get('use_gui') == 'false'
    assert Path(root.find('runtime').get('world_path')).is_file()
    assert ET.parse(directory / 'world.world').getroot().find('world/state') is None
    assert (directory / 'world.original.world').read_bytes() == (SOURCE / 'sjtu_drone_description/worlds/farm_no_animals.world').read_bytes()
    assert manifest['entities'][0]['name'] == '/simple_drone0'
    assert manifest['scenario']['speeds']['drone_linear_mps'] == .7
    assert all(path.read_bytes() == original[name] for name, path in sources().items())


def test_snapshots_load_with_production_parsers(tmp_path):
    import pytest
    cv2 = pytest.importorskip('cv2')
    from cow_pkg.config import load_cow_config
    from sjtu_drone_control.config import load_drone_control_config
    from yolo_pkg.config import load_yolo_config
    configuration = defaults(sources())
    snapshot(tmp_path / 'run', configuration, sources(),
             lambda *_: SOURCE / 'sjtu_drone_description/worlds/farm_no_animals.world')
    directory = tmp_path / 'run/config'
    assert load_cow_config(directory / 'cow_config.xml').linear_velocity_mps == configuration.speeds.cow_linear_mps
    assert load_drone_control_config(directory / 'drone_control_config.xml').max_linear_velocity_mps == configuration.speeds.drone_linear_mps
    assert not load_yolo_config(directory / 'yolo_config.xml').show_annotated_camera
    grid = cv2.FileStorage(str(directory / 'grid_config.xml'), cv2.FILE_STORAGE_READ)
    assert grid.isOpened()
    assert grid.getNode('cell_size_m').real() == float(ET.parse(sources()['grid_config.xml']).getroot().find('cell_size_m').text)
    grid.release()
    # Loading the launch file also verifies compatibility with this ROS distro.
    import runpy
    launch = runpy.run_path(str(SOURCE / 'sjtu_drone_bringup/launch/bringup.launch.py'))
    parsed = launch['_load_simulation_config'](directory / 'simulation.xml')
    assert parsed['drones'] == configuration.model_dump()['drones']
    assert parsed['goal_radius'] == configuration.goal.radius_m
    assert parsed['final_goal_reached_radius'] == configuration.goal.completion_radius_m


def test_default_radii_and_legacy_scenario_compatibility():
    from simulation_test_pkg.models import Goal
    current = defaults(sources()).goal
    assert (current.radius_m, current.completion_radius_m, current.extended_radius_m) == (3, 5, 5)
    legacy = Goal(x=0, y=0, radius_m=2)
    assert legacy.radius_m == legacy.completion_radius_m == 2
    import pytest
    from pydantic import ValidationError
    with pytest.raises(ValidationError):
        Goal(x=0, y=0, radius_m=None)


def test_independent_radius_validation():
    import pytest
    from pydantic import ValidationError
    from simulation_test_pkg.models import Goal
    for change in ({'completion_radius_m': 0}, {'completion_radius_m': float('nan')},
                   {'radius_m': -1}, {'extended_radius_m': 3}, {'extended_radius_m': 2}):
        with pytest.raises(ValidationError):
            Goal.model_validate(dict(x=0, y=0, radius_m=3, completion_radius_m=5,
                                     extended_radius_m=5) | change)


def test_legacy_rerun_preserves_frozen_shared_and_extended_radii(tmp_path):
    from fastapi.testclient import TestClient
    from simulation_test_pkg.server import create_app
    from simulation_test_pkg.supervisor import submit_batch
    app = create_app(tmp_path, manage_worker=False)
    original = defaults(sources())
    original.goal.completion_radius_m = original.goal.radius_m
    original.goal.extended_radius_m = 8
    submit_batch(app.state.store, {'items': [{'scenario': original.model_dump()}]},
                 lambda directory, scenario: snapshot(directory, scenario, sources(),
                     lambda *_: SOURCE / 'sjtu_drone_description/worlds/farm_no_animals.world'))
    previous = app.state.store.runs()[0]
    legacy = original.model_dump()
    del legacy['goal']['completion_radius_m']
    del legacy['goal']['extended_radius_m']
    app.state.store.update(previous['id'], {'scenario': legacy, 'status': 'success'})
    old_file = app.state.store.run_dir(previous['id']) / 'config/simulation.xml'
    tree = ET.parse(old_file)
    del tree.getroot().find('herding').attrib['goal_radius_m']
    tree.write(old_file)
    with TestClient(app) as client:
        response = client.post('/api/runs/' + previous['id'] + '/rerun')
        assert response.status_code == 202, response.text
    repeated = next(r for r in app.state.store.runs() if r['id'] != previous['id'])
    assert repeated['scenario']['goal'] == dict(legacy['goal'], completion_radius_m=3, extended_radius_m=8)
    frozen = ET.parse(app.state.store.run_dir(repeated['id']) / 'config/grid_config.xml').getroot()
    assert float(frozen.find('goal_quiet_radius_m').text) == 8
