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
    params_file = base / 'config' / 'config.yaml'

    # List contents of the directory to debug
    
    holoocean = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('holoocean_main'),
                'launch',
                'holoocean_launch.py'
            )
        ])
    )

    command = launch_ros.actions.Node(
        name='command_node',
        package='holoocean_examples',
        namespace='holoocean',
        executable='command_node',  
        output='screen',
        parameters=[params_file]  
    )

    return LaunchDescription([
        holoocean,
        command,
    ])


