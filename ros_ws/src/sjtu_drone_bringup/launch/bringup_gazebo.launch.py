#!/usr/bin/env python3

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    model_namespace = LaunchConfiguration('model_ns').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)

    description_share = get_package_share_directory(
        'sjtu_drone_description'
    )
    bringup_share = get_package_share_directory('sjtu_drone_bringup')
    xacro_path = os.path.join(
        description_share,
        'urdf',
        'sjtu_drone.urdf.xacro',
    )
    vehicle_config_path = os.path.join(
        bringup_share,
        'config',
        'drone.yaml',
    )

    robot_description_config = xacro.process_file(
        xacro_path,
        mappings={'params_path': vehicle_config_path},
    )
    namespace_tags = robot_description_config.getElementsByTagName(
        'namespace'
    )
    for tag in namespace_tags:
        if tag.firstChild:
            tag.firstChild.nodeValue = model_namespace
        else:
            tag.appendChild(
                robot_description_config.createTextNode(model_namespace)
            )
    robot_description = robot_description_config.toxml()

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=model_namespace,
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'robot_description': robot_description,
                    'frame_prefix': model_namespace + '/',
                }
            ],
            arguments=[robot_description],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=model_namespace,
            output='screen',
        ),
        Node(
            package='sjtu_drone_bringup',
            executable='spawn_drone_script',
            arguments=[robot_description, model_namespace, x, y],
            namespace=model_namespace,
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=model_namespace,
            arguments=[
                '0', '0', '0', '0', '0', '0',
                'world', f'{model_namespace}/odom',
            ],
            output='screen',
        ),
        Node(
            package='sjtu_drone_control',
            executable='drone_position_control',
            name='sjtu_drone_control',
            namespace=model_namespace,
            parameters=[{'drone_namespace': model_namespace}],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('x'),
            DeclareLaunchArgument('y'),
            DeclareLaunchArgument('model_ns'),
            OpaqueFunction(function=_launch_setup),
        ]
    )
