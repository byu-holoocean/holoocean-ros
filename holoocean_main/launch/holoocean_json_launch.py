## Holocean launch file 
# Author: Braden Meyers

from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

def generate_launch_description():
    print('Launching HoloOcean Vehicle Simulation')

    # Set log level
    log_level = 'info'

    base = Path(get_package_share_directory('holoocean_main'))
    params_file = base / 'config' / 'scenario.json'


    # List contents of the directory to debug
    
    assert params_file.is_file(), f"Params file {params_file} does not exist"

    holoocean_namespace = 'holoocean'

    holoocean_main_node = launch_ros.actions.Node(
        name='holoocean_node',
        package='holoocean_main',
        executable='holoocean_node',  
        namespace=holoocean_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'params_file': str(params_file)}]  # Pass parameters in the correct format
    )

    # Define the command node
    command_node = launch_ros.actions.Node(
        name='command_node',
        package='holoocean_main',
        executable='command_node',  # Assuming the executable is command_node.py
        namespace=holoocean_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'params_file': str(params_file)}]  # Pass parameters in the correct format
    )


    return LaunchDescription([
        holoocean_main_node,
        command_node                              
    ])


#Do I need this main function? 
if __name__ == '__main__':
    generate_launch_description()
