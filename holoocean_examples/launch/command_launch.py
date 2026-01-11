## Holocean launch file 
# Author: Braden Meyers

from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from pathlib import Path

def generate_launch_description():
    print('Launching HoloOcean Vehicle Simulation')


    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=str(
            Path(
                os.path.join(
                    get_package_share_directory('holoocean_examples'),
                    'config/config.yaml'
                )
            )
        ),
        description='Full path to the ROS2 parameters file to use for the node',
    )
    params_file = LaunchConfiguration('params_file')

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
        declare_params_file,
        holoocean,
        command,
    ])


