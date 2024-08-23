## Holocean launch file 
# Author: Braden Meyers

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
# from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

def generate_launch_description():
    print('Launching HoloOcean Vehicle Simulation')

    # Set log level
    log_level = 'info'

    base = Path(get_package_share_directory('holoocean_main'))
    params_file = base / 'config' / 'config.yaml'

    log_dir = os.path.join(os.getenv('HOME'), 'ros2ws', 'log')
    # List contents of the directory to debug
    
    holoocean_namespace = 'holoocean'

    holoocean_main_node = launch_ros.actions.Node(
        name='torpedo_node',
        package='holoocean_main',
        executable='torpedo_node',  
        namespace=holoocean_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'params_file': str(params_file)}],  # Pass parameters in the correct format
        remappings=[
                ('/holoocean/ControlCommand', '/control_command'),
            ]    
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

    # Define the command node
    state_est_node = launch_ros.actions.Node(
        name='state_estimation',
        package='holoocean_main',
        executable='state_estimate',  # Assuming the executable is command_node.py
        namespace=holoocean_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'params_file': str(params_file)}],  # Pass parameters in the correct format
    )


    return LaunchDescription([
        holoocean_main_node,
        command_node,
        state_est_node ,
        # rosbag                            
    ])

