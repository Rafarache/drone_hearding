# Authors: Abdulkadir Ture
# Github : abdulkadrtr

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction

# 1. Define a function that takes 'context' and converts the value
def launch_setup(context, *args, **kwargs):
    model_folder = 'turtlebot3_burger'
    robot_desc_path = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "urdf", "turtlebot3_burger.urdf")
    urdf_path = os.path.join(get_package_share_directory('cow_pkg'),'models',model_folder,'model.sdf')
    with open(robot_desc_path, 'r') as infp:
        robot_desc = infp.read()

    name_default = "cow"
    number_of_cows = LaunchConfiguration('number_of_cows').perform(context)
    array = []

    for i in range(int(number_of_cows)):
        name = name_default + str(i)

        spawn_robot = Node(
            package='gazebo_ros', 
            executable='spawn_entity.py', 
            arguments=[
                '-entity', name, 
                '-file', urdf_path, 
                '-x', str(i +2)+'.0', 
                '-y', str(i +2)+'.0', 
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

    return array

def generate_launch_description():
    declare_number_arg = DeclareLaunchArgument(
        'number_of_cows',
        default_value='5',
        description='An example number argument'
    )

    opaque_function_action = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        declare_number_arg,
        opaque_function_action
    ])
