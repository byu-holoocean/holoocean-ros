## Holocean launch file 
# Author: Braden Meyers

from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

def generate_launch_description():
    print('Launching HoloOcean Vehicle Simulation')

    package_dir = Path(get_package_share_directory('holoocean_main'))
    params_file = os.path.join(package_dir, 'config/ros_params.yaml')

    holoocean_namespace = 'holoocean'

    holoocean_main_node = launch_ros.actions.Node(
        name='holoocean_node',
        package='holoocean_main',
        executable='holoocean_node',  
        namespace=holoocean_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[params_file]  # Pass parameters in the correct format
    )


    return LaunchDescription([
        holoocean_main_node,
    ])

