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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xml.etree.ElementTree as ET
import xacro

def launch_setup(context, *args, **kwargs):
    model_ns = LaunchConfiguration('model_ns').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    xacro_file_name = "sjtu_drone.urdf.xacro"
    xacro_file = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "urdf", xacro_file_name
    )
    yaml_file_path = os.path.join(
        get_package_share_directory('sjtu_drone_bringup'),
        'config', 'drone.yaml'
    )   
    
    robot_description_config = xacro.process_file(xacro_file, mappings={"params_path": yaml_file_path})
    robot_desc = robot_description_config.toxml()

    robot_description_config = xacro.process_file(xacro_file, mappings={"params_path": yaml_file_path})
    namespace_tags = robot_description_config.getElementsByTagName('namespace')

    for tag in namespace_tags:
        if tag.firstChild:
            tag.firstChild.nodeValue = model_ns
        else:
            new_text = robot_description_config.createTextNode(model_ns)
            tag.appendChild(new_text)

    robot_desc = robot_description_config.toxml()

    #print(robot_desc)

    array = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=model_ns,
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc, "frame_prefix": model_ns + "/"}],
            arguments=[robot_desc]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=model_ns,
            output='screen'
        ),

        Node(
            package="sjtu_drone_bringup",
            executable="spawn_drone_script",
            arguments=[robot_desc, model_ns, x, y],
            namespace=model_ns,
            output="screen"
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace=model_ns,
            arguments=["0", "0", "0", "0", "0", "0", "world", f"{model_ns}/odom"],
            output="screen"
        ),

        Node(
            package="sjtu_drone_control",
            executable="drone_position_control",
            arguments=[model_ns],
            name='sjtu_drone_control',
            output="screen"
        ),
    ]

    return array

def generate_launch_description():
    use_gui = DeclareLaunchArgument("use_gui", default_value="true", choices=["true", "false"],
                                    description="Whether to execute gzclient")
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file_default = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "worlds", "farm_no_animals.world"
    )

    world_file = LaunchConfiguration('world', default=world_file_default)

    world = DeclareLaunchArgument(
        name='world',
        default_value=world_file_default,
        description='Full path to world file to load'
    )

    declare_x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='X position of the drone'
    )

    declare_y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Y position of the drone'
    )

    declare_model_ns_arg = DeclareLaunchArgument(
        'model_ns',
        default_value='drone',
        description='Namespace of the drone model'
    )

    def launch_gzclient(context, *args, **kwargs):
        if context.launch_configurations.get('use_gui') == 'true':
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
                ),
                launch_arguments={'verbose': 'true'}.items()
            )]
        return []

    opaque_function_action = OpaqueFunction(function=launch_setup)

    # Log the world file being used
    print("Loading world file: ", world_file_default)

    return LaunchDescription([
        world,
        use_gui,
        declare_x_arg,
        declare_y_arg,
        declare_model_ns_arg,
        opaque_function_action,
    ])
