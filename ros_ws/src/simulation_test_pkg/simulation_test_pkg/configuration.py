"""Freeze effective inputs without rewriting the installed simulation defaults."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
from .models import Scenario
from .storage import atomic_json

FILES = {'simulation.xml': ('sjtu_drone_bringup', 'simulation.xml'),
         'cow_config.xml': ('cow_pkg', 'cow_config.xml'),
         'drone_control_config.xml': ('sjtu_drone_control', 'drone_control_config.xml'),
         'grid_config.xml': ('grid_visualizer_pkg', 'grid_config.xml'),
         'yolo_config.xml': ('yolo_pkg', 'yolo_config.xml'),
         'drone.yaml': ('sjtu_drone_bringup', 'drone.yaml')}


def config_sources():
    from ament_index_python.packages import get_package_share_directory
    return {name: Path(get_package_share_directory(package)) / 'config' / filename
            for name, (package, filename) in FILES.items()}


def defaults(sources=None):
    sources = sources or config_sources()
    simulation = ET.parse(sources['simulation.xml']).getroot()
    cow = ET.parse(sources['cow_config.xml']).getroot().find('motion')
    drone = ET.parse(sources['drone_control_config.xml']).getroot().find('motion')
    field = simulation.find('map').attrib
    goal = simulation.find('herding').attrib
    grid = ET.parse(sources['grid_config.xml']).getroot()
    return Scenario(name='Default field',
                    map={k: float(field[k]) for k in ('center_x', 'center_y', 'width_m', 'height_m')},
                    goal={'x': float(goal['goal_x']), 'y': float(goal['goal_y']),
                          'radius_m': float(goal.get('goal_radius_m', goal['final_goal_reached_radius_m'])),
                          'completion_radius_m': float(goal['final_goal_reached_radius_m']),
                          'extended_radius_m': float(grid.find('goal_quiet_radius_m').text)},
                    drones=[{'x': float(p.get('x')), 'y': float(p.get('y'))} for p in simulation.find('drones')],
                    cows=[{k: float(p.get(k)) for k in ('x', 'y', 'z')} for p in simulation.find('cows')],
                    speeds={'cow_linear_mps': float(cow.get('linear_velocity_mps')),
                            'cow_angular_radps': float(cow.get('max_angular_velocity_radps')),
                            'drone_linear_mps': float(drone.get('max_linear_velocity_mps')),
                            'drone_angular_radps': float(drone.get('max_angular_velocity_radps'))})


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def snapshot(run_dir, scenario, sources=None, resolve_world=None):
    sources = sources or config_sources()
    if resolve_world is None:
        from ament_index_python.packages import get_package_share_directory
        resolve_world = lambda package, relative: Path(get_package_share_directory(package)) / relative
    run_dir = Path(run_dir)
    target = run_dir / 'config'
    target.mkdir(parents=True)
    for name, source in sources.items():
        shutil.copyfile(source, target / name)
    scenario = Scenario.model_validate(scenario).model_dump()
    trees = {name: ET.parse(target / name) for name in FILES if name.endswith('.xml')}
    root = trees['simulation.xml'].getroot()
    runtime = root.find('runtime')
    world = resolve_world(runtime.get('world_package'), runtime.get('world_path'))
    shutil.copyfile(world, target / 'world.original.world')
    # Gazebo-saved worlds can contain an archival <state> with a large sim_time
    # and references to deleted models. A fresh experiment must not restore it:
    # that clock jump can expire spawn-service requests during startup.
    world_tree = ET.parse(world)
    for element in world_tree.getroot().findall('world'):
        for state in list(element.findall('state')):
            element.remove(state)
    world_tree.write(target / 'world.world', encoding='utf-8', xml_declaration=True)
    runtime.set('world_path', str((target / 'world.world').resolve()))
    views = scenario.get('visualization', {})
    runtime.set('use_gui', 'true' if views.get('gazebo') else 'false')
    for key, value in scenario['map'].items():
        root.find('map').set(key, str(value))
    for key, value in [('goal_x', scenario['goal']['x']), ('goal_y', scenario['goal']['y']),
                       ('goal_radius_m', scenario['goal']['radius_m']),
                       ('final_goal_reached_radius_m', scenario['goal']['completion_radius_m'])]:
        root.find('herding').set(key, str(value))
    for group, tag in [('drones', 'drone'), ('cows', 'cow')]:
        parent = root.find(group)
        parent.clear()
        for point in scenario[group]:
            ET.SubElement(parent, tag, {k: str(v) for k, v in point.items()})
    speeds = scenario['speeds']
    cow = trees['cow_config.xml'].getroot().find('motion')
    cow.set('linear_velocity_mps', str(speeds['cow_linear_mps']))
    cow.set('max_angular_velocity_radps', str(speeds['cow_angular_radps']))
    drone = trees['drone_control_config.xml'].getroot().find('motion')
    drone.set('max_linear_velocity_mps', str(speeds['drone_linear_mps']))
    drone.set('max_angular_velocity_radps', str(speeds['drone_angular_radps']))
    grid = trees['grid_config.xml'].getroot()
    quiet = grid.find('goal_quiet_radius_m')
    quiet.text = str(scenario['goal']['extended_radius_m'])
    for name, setting in (('show_vector_grid', 'vectors'), ('show_occupancy_grid', 'occupancy'),
                          ('show_coverage_map', 'visited')):
        grid.find(name).text = '1' if views.get(setting) else '0'
    detector = trees['yolo_config.xml'].getroot()
    for name, setting in (('show_annotated_camera', 'camera'), ('show_realtime_track_plot', 'tracker')):
        detector.find('display').set(name, 'true' if views.get(setting) else 'false')
    weight = Path(detector.find('detector').get('model_path')).expanduser()
    if not weight.is_absolute():
        weight = Path.cwd() / weight
    weight_info = {'path': str(weight), 'sha256': sha256(weight) if weight.is_file() else None}
    if weight.is_file():
        detector.find('detector').set('model_path', str(weight.resolve()))
    for name, tree in trees.items():
        tree.write(target / name, encoding='utf-8', xml_declaration=True)
    interface = root.find('interfaces')
    entities = [{'name': f"{interface.get('drone_namespace_prefix')}{i}", 'kind': 'drone', 'index': i}
                for i in range(len(scenario['drones']))]
    entities += [{'name': f"{interface.get('cow_name_prefix')}{i}", 'kind': 'cow', 'index': i}
                 for i in range(len(scenario['cows']))]
    manifest = {'scenario': scenario, 'entities': entities,
                'cow_positions_topic': interface.get('cow_positions_topic'),
                'detector_weights': weight_info,
                'configuration_sha256': {p.name: sha256(p) for p in target.iterdir()},
                'git_revision': os.environ.get('SIM_TEST_GIT_REV', 'unknown'),
                'git_dirty': os.environ.get('SIM_TEST_GIT_DIRTY', 'unknown'),
                'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
                'world_archival_state_removed': True, 'search_telemetry_version': 1,
                'live_visualization_controls': True}
    provenance = run_dir.parent.parent / 'checkout_provenance.json'
    if provenance.exists():
        manifest['checkout'] = json.loads(provenance.read_text())
    atomic_json(run_dir / 'manifest.json', manifest)
    return manifest
