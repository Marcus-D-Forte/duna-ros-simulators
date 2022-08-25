"""Launch Gazebo server and client with command line arguments."""
"""Spawn robot from URDF file."""


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

def generate_launch_description():

    world_file_name = 'truckscan.world'
    urdf_file = 'truckscan.urdf'

    world = os.path.join(get_package_share_directory(
        'duna_simulation'), 'worlds', world_file_name)

    urdf = os.path.join(get_package_share_directory(
        'duna_simulation'), 'urdf', urdf_file)

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_desc}])

    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so', 
            '-s', 'libgazebo_ros_factory.so'],
            output='screen')

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', '2dlidar', '-topic', '/robot_description'],
                        output='screen')

    return LaunchDescription([
        gazebo,
        start_robot_state_publisher_cmd,
        spawn_entity
    ])
