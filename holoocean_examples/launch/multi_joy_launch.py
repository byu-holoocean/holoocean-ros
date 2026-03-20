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
    params_file = base / 'config' / 'multi_joy_config.yaml'

    holoocean_namespace = 'holoocean'


    # TODO make the parameters file a pass in argument to the launch file
    joy_node_wired = launch_ros.actions.Node(
        package='joy_linux',
        name='joy_node_wired',
        namespace=holoocean_namespace,
        executable='joy_linux_node',
        output='screen',
        parameters=[params_file], 
        remappings=[('joy', 'joy_wired')]
    )
    
    joy_holo_wired = launch_ros.actions.Node(
        name='joy_holoocean_wired',
        package='holoocean_examples',
        namespace=holoocean_namespace,
        executable='joy_holoocean',  
        output='screen',
        parameters=[params_file], 
        remappings=[('joy', 'joy_wired')] 
    )

    # Wireless joystick nodes
    joy_node = launch_ros.actions.Node(
        package='joy_linux',
        name='joy_node',
        namespace=holoocean_namespace,
        executable='joy_linux_node',
        output='screen',
        parameters=[params_file] 
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

    camera_hud_node = launch_ros.actions.Node(
        name='camera_hud',
        package='holoocean_examples',
        executable='camera_hud',
        namespace=holoocean_namespace,
        output='screen',
        parameters=[params_file],
    )
    camera_hud_node_sv = launch_ros.actions.Node(
        name='camera_hud_sv',
        package='holoocean_examples',
        executable='camera_hud',
        namespace=holoocean_namespace,
        output='screen',
        parameters=[params_file],
    )
    rqt_image_sv_node = launch_ros.actions.Node(
        name='sv_image_view',
        package='rqt_image_view',
        executable='rqt_image_view',
        namespace=holoocean_namespace,
        output='screen',   
        arguments=['/holoocean/auv1/CameraHUD'],
    )
    rqt_image_auv_node = launch_ros.actions.Node(
        name='auv_image_view',
        package='rqt_image_view',
        executable='rqt_image_view',
        namespace=holoocean_namespace,
        output='screen',   
        arguments=['/holoocean/auv0/CameraHUD'],
    )

    return LaunchDescription([
        holoocean_main_node,
        joy_holo,
        joy_node,
        joy_holo_wired,
        joy_node_wired,
        camera_hud_node,
        camera_hud_node_sv,
        rqt_image_sv_node,
        rqt_image_auv_node  
    ])


