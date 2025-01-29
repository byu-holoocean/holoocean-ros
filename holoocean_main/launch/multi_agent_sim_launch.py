from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            name='multi_agent_sim',  # Node name
            package='holoocean_main',  # package name
            executable='multi_agent_sim_node',  # The executable name (e.g., the Python script's entry point)
            output='screen',  # Output logs to the screen
            emulate_tty=True,
            parameters=[]  # Add parameters if needed
        )
    ])
