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
    manifest = snapshot(tmp_path / 'run', scenario, sources(),
                        lambda *_: SOURCE / 'sjtu_drone_description/worlds/farm_no_animals.world')
    directory = tmp_path / 'run/config'
    assert ET.parse(directory / 'cow_config.xml').getroot().find('motion').get('linear_velocity_mps') == '0.25'
    assert ET.parse(directory / 'drone_control_config.xml').getroot().find('motion').get('max_linear_velocity_mps') == '0.7'
    assert float(ET.parse(directory / 'grid_config.xml').getroot().find('goal_quiet_radius_m').text) > 9
    root = ET.parse(directory / 'simulation.xml').getroot()
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
