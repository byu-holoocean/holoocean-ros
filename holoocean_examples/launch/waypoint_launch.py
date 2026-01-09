## Holocean launch file 
# Author: Braden Meyers

from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from pathlib import Path

def generate_launch_description():
    print('Launching HoloOcean Vehicle Simulation')

    base = Path(get_package_share_directory('holoocean_examples'))
    params_file = base / 'config' / 'waypoint_config.yaml'
    print('Params file: ', params_file)
    holoocean_namespace = 'holoocean'

    # List contents of the directory to debug
    
    holoocean_main_node = launch_ros.actions.Node(
        name='holoocean_node',
        package='holoocean_main',
        executable='holoocean_node',
        namespace=holoocean_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
    )

    command = launch_ros.actions.Node(
        name='waypoint_follower',
        package='holoocean_examples',
        namespace=holoocean_namespace,
        executable='waypoint_follower',  
        output='screen',
        parameters=[params_file]  
    )

    return LaunchDescription([
        holoocean_main_node,
        command,
    ])


