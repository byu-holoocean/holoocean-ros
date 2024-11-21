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
    params_file = base / 'config' / 'config.yaml'

    # List contents of the directory to debug
    
    holoocean_namespace = 'holoocean'
    output_namespace = '/coug3'

    fins_node = launch_ros.actions.Node(
        name='fins_node',
        package='holoocean_main',
        executable='moos_fins_node',  
        namespace=holoocean_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'params_file': str(params_file)}], 
        remappings=[
            ('/holoocean/ControlCommand', f'{output_namespace}/controls/command'),
            ('/holoocean/DepthSensor', f'{output_namespace}/depth_data'),
            ('/holoocean/ModemIMU', f'{output_namespace}/modem_imu'),
            ('/holoocean/GPSSensor', f'{output_namespace}/gps_odom'),
            ('/holoocean/reset_holoocean', f'{output_namespace}/reset_holoocean'),
            ]
    )


    # Define the command node
    state_est_node = launch_ros.actions.Node(
        name='state_estimation',
        package='holoocean_main',
        executable='state_estimate',  
        namespace=holoocean_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'params_file': str(params_file)}],
        remappings=[
            ('/holoocean/dead_reckon', f'{output_namespace}/dvl_dead_reckoning')
        ]  
    )



    return LaunchDescription([
        fins_node,
        state_est_node,
    ])


#Do I need this main function? 
if __name__ == '__main__':
    generate_launch_description()
