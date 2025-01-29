from launch import LaunchDescription
from launch_ros.actions import Node

holoocean_namespace = 'holoocean'


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='holoocean_main',
            executable='sv_waypoint_navigator',
            name='sv_waypoint_navigator',
            output='screen',
            namespace=holoocean_namespace,
            emulate_tty=True,
        )
    ])