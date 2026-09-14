#!/usr/bin/env python3

import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _required_element(root, name, source_path):
    element = root.find(name)
    if element is None:
        raise ValueError(f'Missing <{name}> section in {source_path}')
    return element


def _required_text(element, attribute, source_path):
    value = element.get(attribute)
    if value is None:
        raise ValueError(
            f'Missing attribute {attribute!r} in {source_path}'
        )
    return value.strip()


def _required_float(element, attribute, source_path):
    raw = _required_text(element, attribute, source_path)
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError(
            f'Attribute {attribute!r} must be numeric in {source_path}'
        ) from exc
    if not math.isfinite(value):
        raise ValueError(
            f'Attribute {attribute!r} must be finite in {source_path}'
        )
    return value


def _required_bool(element, attribute, source_path):
    raw = _required_text(element, attribute, source_path).lower()
    if raw not in {'true', 'false'}:
        raise ValueError(
            f'Attribute {attribute!r} must be true or false in {source_path}'
        )
    return raw == 'true'


def _parse_positions(parent, element_name, source_path, include_z):
    positions = []
    for index, element in enumerate(parent.findall(element_name)):
        position = {
            'x': _required_float(element, 'x', source_path),
            'y': _required_float(element, 'y', source_path),
        }
        if include_z:
            position['z'] = _required_float(element, 'z', source_path)
        positions.append(position)

    if not positions:
        raise ValueError(
            f'{source_path} must contain at least one <{element_name}>'
        )
    return positions


def _load_simulation_config(source_path):
    try:
        root = ET.parse(source_path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise RuntimeError(
            f'Unable to load simulation configuration {source_path}: {exc}'
        ) from exc
    if root.tag != 'simulation':
        raise ValueError(
            f'Expected <simulation> root in {source_path}, got <{root.tag}>'
        )

    runtime = _required_element(root, 'runtime', source_path)
    map_element = _required_element(root, 'map', source_path)
    interfaces = _required_element(root, 'interfaces', source_path)
    herding = _required_element(root, 'herding', source_path)
    drones_element = _required_element(root, 'drones', source_path)
    cows_element = _required_element(root, 'cows', source_path)

    config = {
        'source_path': source_path,
        'world_package': _required_text(
            runtime, 'world_package', source_path
        ),
        'world_path': _required_text(runtime, 'world_path', source_path),
        'use_gui': _required_bool(runtime, 'use_gui', source_path),
        'controller': _required_text(runtime, 'controller', source_path),
        'fixed_frame': runtime.get('fixed_frame', '').strip(),
        'map_center_x': _required_float(
            map_element, 'center_x', source_path
        ),
        'map_center_y': _required_float(
            map_element, 'center_y', source_path
        ),
        'map_width_m': _required_float(
            map_element, 'width_m', source_path
        ),
        'map_height_m': _required_float(
            map_element, 'height_m', source_path
        ),
        'drone_namespace_prefix': _required_text(
            interfaces, 'drone_namespace_prefix', source_path
        ),
        'cow_name_prefix': _required_text(
            interfaces, 'cow_name_prefix', source_path
        ),
        'cow_positions_topic': _required_text(
            interfaces, 'cow_positions_topic', source_path
        ),
        'drone_approach_radius': _required_float(
            herding, 'drone_approach_radius_m', source_path
        ),
        'push_point_margin': _required_float(
            herding, 'push_point_margin_m', source_path
        ),
        'drones': _parse_positions(
            drones_element, 'drone', source_path, include_z=False
        ),
        'cows': _parse_positions(
            cows_element, 'cow', source_path, include_z=True
        ),
    }

    if config['controller'] not in {'keyboard', 'joystick'}:
        raise ValueError(
            f"controller must be keyboard or joystick in {source_path}"
        )
    if config['map_width_m'] <= 0.0 or config['map_height_m'] <= 0.0:
        raise ValueError(f'Map dimensions must be positive in {source_path}')
    if config['drone_approach_radius'] <= 0.0:
        raise ValueError(
            f'drone_approach_radius_m must be positive in {source_path}'
        )
    if config['push_point_margin'] < 0.0:
        raise ValueError(
            f'push_point_margin_m must be nonnegative in {source_path}'
        )
    for name in (
        'world_package',
        'world_path',
        'drone_namespace_prefix',
        'cow_name_prefix',
        'cow_positions_topic',
    ):
        if not config[name]:
            raise ValueError(f'{name} cannot be empty in {source_path}')

    min_x = config['map_center_x'] - config['map_width_m'] / 2.0
    max_x = config['map_center_x'] + config['map_width_m'] / 2.0
    min_y = config['map_center_y'] - config['map_height_m'] / 2.0
    max_y = config['map_center_y'] + config['map_height_m'] / 2.0
    for entity_type in ('drones', 'cows'):
        for index, position in enumerate(config[entity_type]):
            if not (
                min_x <= position['x'] <= max_x
                and min_y <= position['y'] <= max_y
            ):
                raise ValueError(
                    f'{entity_type}[{index}] position is outside the map '
                    f'in {source_path}'
                )

    return config


def _teleop_node(namespace, controller):
    if controller == 'joystick':
        return Node(
            package='sjtu_drone_control',
            executable='teleop_joystick',
            namespace=namespace,
            output='screen',
        )
    return Node(
        package='sjtu_drone_control',
        executable='teleop',
        namespace=namespace,
        output='screen',
        prefix='xterm -e',
    )


def _launch_setup(context):
    source_path = LaunchConfiguration(
        'simulation_config_file'
    ).perform(context)
    config = _load_simulation_config(source_path)

    bringup_share = get_package_share_directory('sjtu_drone_bringup')
    gazebo_share = get_package_share_directory('gazebo_ros')
    world_share = get_package_share_directory(config['world_package'])
    world_file = os.path.join(world_share, config['world_path'])
    if not os.path.isfile(world_file):
        raise ValueError(f'World file does not exist: {world_file}')

    number_of_drones = len(config['drones'])
    number_of_cows = len(config['cows'])
    namespace_prefix = config['drone_namespace_prefix']
    cow_name_prefix = config['cow_name_prefix']

    print(
        f"Loaded simulation configuration: {source_path}; "
        f"drones={number_of_drones}, cows={number_of_cows}, "
        f"map={config['map_width_m']}x{config['map_height_m']} m"
    )

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={
                'world': world_file,
                'verbose': 'true',
                'extra_gazebo_args': 'verbose',
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=(
                ['-d', os.path.join(bringup_share, 'rviz', 'rviz.rviz')]
                + (
                    ['--fixed-frame', config['fixed_frame']]
                    if config['fixed_frame']
                    else []
                )
            ),
            output='screen',
        ),
    ]

    if config['use_gui']:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')
                ),
                launch_arguments={'verbose': 'true'}.items(),
            )
        )

    cow_model_folder = 'turtlebot3_burger'
    cow_model_path = os.path.join(
        get_package_share_directory('cow_pkg'),
        'models',
        cow_model_folder,
        'model.sdf',
    )
    robot_description_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        'turtlebot3_burger.urdf',
    )
    with open(robot_description_path, 'r', encoding='utf-8') as stream:
        cow_robot_description = stream.read()

    for index, position in enumerate(config['cows']):
        name = cow_name_prefix + str(index)
        actions.extend(
            [
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', name,
                        '-file', cow_model_path,
                        '-x', str(position['x']),
                        '-y', str(position['y']),
                        '-z', str(position['z']),
                        '-robot_namespace', name,
                    ],
                    output='screen',
                ),
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    namespace=name,
                    output='screen',
                    parameters=[
                        {
                            'frame_prefix': name + '/',
                            'use_sim_time': True,
                            'robot_description': cow_robot_description,
                        }
                    ],
                ),
            ]
        )

    shared_parameters = {
        'drone_namespace_prefix': namespace_prefix,
        'cow_name_prefix': cow_name_prefix,
        'number_of_drones': number_of_drones,
        'number_of_cows': number_of_cows,
        'drone_approach_radius': config['drone_approach_radius'],
        'push_point_margin': config['push_point_margin'],
    }
    actions.append(
        Node(
            package='cow_pkg',
            executable='repeller',
            name='cow_pkg',
            parameters=[shared_parameters],
            output='screen',
        )
    )
    actions.append(
        Node(
            package='yolo_pkg',
            executable='yolo_subscriber',
            name='yolo_pkg',
            parameters=[
                {
                    'drone_namespace_prefix': namespace_prefix,
                    'number_of_drones': number_of_drones,
                    'cow_positions_topic': config['cow_positions_topic'],
                }
            ],
            output='screen',
        )
    )

    drone_launch = os.path.join(
        bringup_share,
        'launch',
        'bringup_gazebo.launch.py',
    )
    for index, position in enumerate(config['drones']):
        namespace = namespace_prefix + str(index)
        actions.extend(
            [
                Node(
                    package='joy',
                    executable='joy_node',
                    name='joy',
                    namespace=namespace,
                    output='screen',
                ),
                _teleop_node(namespace, config['controller']),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(drone_launch),
                    launch_arguments={
                        'x': str(position['x']),
                        'y': str(position['y']),
                        'model_ns': namespace,
                    }.items(),
                ),
                Node(
                    package='grid_visualizer_pkg',
                    executable='grid_visualizer_node',
                    namespace=namespace,
                    name=f'grid{index}',
                    parameters=[
                        {
                            'drone_index': index,
                            'total_drones': number_of_drones,
                            'global_map_center_x': config['map_center_x'],
                            'global_map_center_y': config['map_center_y'],
                            'global_map_width': config['map_width_m'],
                            'global_map_height': config['map_height_m'],
                            'drone_namespace_prefix': namespace_prefix,
                            'cow_positions_topic':
                                config['cow_positions_topic'],
                            'cow_exclusion_radius':
                                config['drone_approach_radius'],
                            'push_point_margin':
                                config['push_point_margin'],
                        }
                    ],
                    output='screen',
                ),
            ]
        )

    return actions


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('sjtu_drone_bringup'),
        'config',
        'simulation.xml',
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'simulation_config_file',
                default_value=default_config,
                description='Path to the authoritative simulation XML file',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
