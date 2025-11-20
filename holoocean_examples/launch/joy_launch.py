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
    params_file = base / 'config' / 'joy_config.yaml'

    holoocean_namespace = 'holoocean'

    # TODO make a flag to install the deps for the examples
    joy_node = launch_ros.actions.Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        output='screen',
    )
    
    joy_holo = launch_ros.actions.Node(
        name='joy_holoocean',
        package='holoocean_examples',
        namespace=holoocean_namespace,
        executable='joy_holoocean',  
        output='screen',
        parameters=[params_file]  
    )

    holoocean_main_node = launch_ros.actions.Node(
        name='holoocean_node',
        package='holoocean_main',
        executable='holoocean_node',
        namespace=holoocean_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
    )

    return LaunchDescription([
        holoocean_main_node,
        joy_holo,
        joy_node,
    ])


