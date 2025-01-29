from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='holoocean_main',
            executable='multi_waypoint_navigator',
            name='multi_waypoint_navigator',
            output='screen',
            emulate_tty=True,
        )
    ])
