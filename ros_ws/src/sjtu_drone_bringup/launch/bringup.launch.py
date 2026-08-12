#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration

def get_teleop_controller(context, *_, **kwargs) -> Node:
    controller = context.launch_configurations["controller"]
    namespace = kwargs["model_ns"]

    print(namespace)

    if controller == "joystick":
        node = Node(
            package="sjtu_drone_control",
            executable="teleop_joystick",
            namespace=namespace,
            output="screen",
        )

    else:
        node = Node(
            package="sjtu_drone_control",
            executable="teleop",
            namespace=namespace,
            output="screen",
            prefix="xterm -e",
        )

    return [node]

def rviz_node_generator(context, rviz_path):
    """Return a Node action for RViz, omitting --fixed-frame if empty."""
    fixed_frame_value = LaunchConfiguration('fixed_frame').perform(context)

    rviz_arguments = ['-d', rviz_path]

    if fixed_frame_value:
        rviz_arguments.extend(['--fixed-frame', fixed_frame_value])

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=rviz_arguments,
            output='screen',
        )
    ]

def cow_launch_description(context, *args, **kwargs):
    model_folder = 'turtlebot3_burger'
    robot_desc_path = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "urdf", "turtlebot3_burger.urdf")
    urdf_path = os.path.join(get_package_share_directory('cow_pkg'),'models',model_folder,'model.sdf')
    with open(robot_desc_path, 'r') as infp:
        robot_desc = infp.read()

    name_default = "cow"
    number_of_cows = LaunchConfiguration('number_of_cows').perform(context)
    number_of_drones = LaunchConfiguration('number_of_drones').perform(context)
    model_ns = "/simple_drone"
    array = []

    for i in range(int(number_of_cows)):
        name = name_default + str(i)

        spawn_robot = Node(
            package='gazebo_ros', 
            executable='spawn_entity.py', 
            arguments=[
                '-entity', name, 
                '-file', urdf_path, 
                '-x', '3.0', 
                '-y', str(i *1)+'.0', 
                '-z', '0.01',
                '-robot_namespace', name,
            ],
            output='screen'
        )

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=name,
            output='screen',
            parameters=[{'frame_prefix': name + '/',
                        'use_sim_time': True,
                        'robot_description': robot_desc}]
        )

        array.append(spawn_robot)
        array.append(robot_state_publisher)

    # array.append(
    #     Node(
    #         package="cow_pkg",
    #         executable="repeller",
    #         arguments=[model_ns, number_of_drones, number_of_cows],
    #         name='cow_pkg',
    #         output="screen"
    #     ),
    # )

    array.append(
        Node(
            package="yolo_pkg",
            executable="yolo_subscriber",
            arguments=[model_ns, number_of_cows],
            name='yolo_pkg',
            output="screen"
        ),
    )

    # array.append(
    #     Node(
    #         package="herding_pkg",
    #         executable="herding_control_my",
    #         arguments=[model_ns, number_of_drones],
    #         name='herding_pkg',
    #         output="screen"
    #     ),
    # )

    return array

def drone_launch_description(context, *args, **kwargs):
    array = []
    model_ns = "/simple_drone"
    sjtu_drone_bringup_path = get_package_share_directory('sjtu_drone_bringup')

    number_of_drones = LaunchConfiguration('number_of_drones').perform(context)

    cow_pos = [
        [3,5],
        [-6,7],
        [-3,2]
    ]

    for i in range(int(number_of_drones)):
        name = model_ns + str(i)

        node = Node(
            package='joy',
            executable='joy_node',
            name='joy',
            namespace=name,
            output='screen',
        )

        func = OpaqueFunction(
            function=get_teleop_controller,
            kwargs={'model_ns': name},
        )

        desc = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sjtu_drone_bringup_path, 'launch', 'bringup_gazebo.launch.py')
            ),
            launch_arguments={
                'x': str(0),
                'y': str(i*2),
                'model_ns': name
            }.items()
        )

        array.append(
        Node(
            package="grid_visualizer_pkg",
            executable="grid_visualizer_node",
            arguments=[name],
            namespace=name,
            name='grid_visualizer_pkg',
            output="screen"
        ),
    )

        array.append(node)
        array.append(func)
        array.append(desc)
    
    return array


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file_default = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "worlds", "farm_no_animals.world"
    )

    world_file = LaunchConfiguration('world', default=world_file_default)

    def launch_gzclient(context, *args, **kwargs):
        if context.launch_configurations.get('use_gui') == 'true':
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
                ),
                launch_arguments={'verbose': 'true'}.items()
            )]
        return []

    sjtu_drone_bringup_path = get_package_share_directory('sjtu_drone_bringup')

    rviz_path = os.path.join(
        sjtu_drone_bringup_path, "rviz", "rviz.rviz"
    )

    declare_number__cows_arg = DeclareLaunchArgument(
        'number_of_cows',
        default_value='3',
        description='Number of cows argument'
    )

    declare_number_drones_arg = DeclareLaunchArgument(
        'number_of_drones',
        default_value='1',
        description='Number of drones argument'
    )

    cow_launch_function_action = OpaqueFunction(function=cow_launch_description)

    drone_launch_function_action = OpaqueFunction(function=drone_launch_description)

    return LaunchDescription([
        DeclareLaunchArgument(
            "controller",
            default_value="keyboard",
            description="Type of controller: keyboard (default) or joystick",
        ),

        DeclareLaunchArgument(
            'fixed_frame',
            default_value='',
            description='If provided, sets the fixed frame in RViz.'
        ),

        OpaqueFunction(
            function=rviz_node_generator,
            kwargs={'rviz_path': rviz_path},
        ),

        declare_number__cows_arg,
        declare_number_drones_arg,
        cow_launch_function_action,
        drone_launch_function_action,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file,
                              'verbose': "true",
                              'extra_gazebo_args': 'verbose'}.items()
        ),

        OpaqueFunction(function=launch_gzclient),

    ])