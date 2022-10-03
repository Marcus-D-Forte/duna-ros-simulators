from http.server import executable
import os
from sys import prefix
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition #1

def generate_launch_description():
    websocketIP = LaunchConfiguration("address", default="localhost")

    rosbridge_websocket_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
    )

    keyboard_node = Node(
        package='duna_vr_navigator',
        executable='keyboard_publisher',
        name='keyboard_publisher',
        output='screen',
        prefix='xterm -e'
    )

    return LaunchDescription(
        [
            keyboard_node,
            rosbridge_websocket_node
        ]
    )