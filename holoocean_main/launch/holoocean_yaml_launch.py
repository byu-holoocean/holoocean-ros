## Holocean launch file 
# Author: Braden Meyers

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

def generate_launch_description():
    print('Launching HoloOcean Vehicle Simulation')

    # Set log level
    log_level = 'info'

    base = Path(get_package_share_directory('holoocean_main'))
    params_file = base / 'config' / 'config.json'


    # List contents of the directory to debug
    

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

    yaml_file = LaunchConfiguration('yaml_file')
    json_file = LaunchConfiguration('json_file')

    yaml_path = DeclareLaunchArgument(
        'yaml_file',
        default_value=os.path.join(get_package_share_directory('holoocean_main'), 'config', 'config.yaml'),
        description='Path to the YAML configuration file'
    )
    json_path = DeclareLaunchArgument(
        'json_file',
        default_value=os.path.join(get_package_share_directory('holoocean_main'), 'config', 'config.json'),
        description='Path to the output JSON configuration file'
    )

    convert_yaml = ExecuteProcess(
        cmd=['python3', os.path.join(get_package_share_directory('holoocean_main'), 'scripts', 'yaml_to_json.py'), yaml_file, json_file],
        output='screen'
    )

    return LaunchDescription([
        yaml_path,
        json_path,
        convert_yaml,
        holoocean_main_node,
        command_node
    ])



#Do I need this main function? 
if __name__ == '__main__':
    generate_launch_description()
