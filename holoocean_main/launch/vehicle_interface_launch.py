## HoloOcean Vehicle Interface Launch File
# Author: Jay (adapted from Braden Meyers)

from launch import LaunchDescription
import launch_ros.actions
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    print("Launching HoloOcean Vehicle Interface Simulation")

    # Set log level
    log_level = 'info'

    # Path to the parameters file (if needed)
    base = Path(get_package_share_directory('holoocean_main'))
    params_file = base / 'config' / 'config.yaml'
    
    log_dir = os.path.join(os.getenv('HOME'), 'ros2_ws', 'log')

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
    
    print("Torpedo node launched.")
    
    # Define the vehicle interface node
    vehicle_interface_node = launch_ros.actions.Node(
        name='vehicle_interface_node',
        package='holoocean_main',
        executable='vehicle_interface_node',  # Name of the Python executable
        namespace=holoocean_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'params_file': str(params_file)}],  # Pass parameters in the correct format
    )
    
    # Define the command node
    state_est_node = launch_ros.actions.Node(
        name='state_estimation',
        package='holoocean_main',
        executable='state_estimate',  
        namespace=holoocean_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'params_file': str(params_file)}],  # Pass parameters in the correct format
    )

    return LaunchDescription([
        holoocean_main_node,
        vehicle_interface_node,
        state_est_node
        # Add other nodes here if needed
    ])



