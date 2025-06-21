## Holocean launch file 
# Author: Braden Meyers

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

def generate_launch_description():
    print('Launching HoloOcean Vehicle Simulation')

    # Declare a launch argument for the parameter file
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=str(
            Path(
                os.path.join(
                    get_package_share_directory('holoocean_main'),
                    'config/ros_params.yaml'
                )
            )
        ),
        description='Full path to the ROS2 parameters file to use for the node',
    )

    # Get the launch configuration for the parameter file
    params_file = LaunchConfiguration('params_file')

    holoocean_namespace = 'holoocean'

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
        declare_params_file,
        holoocean_main_node,
    ])
