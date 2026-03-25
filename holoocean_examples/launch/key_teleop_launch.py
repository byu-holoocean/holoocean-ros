## HoloOcean keyboard teleop launch file
#
# Starts the simulation and a Twist-to-AgentCommand converter node.
# Run teleop_twist_keyboard separately in its own terminal:
#
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
#       --ros-args -r __ns:=/holoocean

from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os


def generate_launch_description():
    print('Launching HoloOcean Keyboard Teleop')

    # Declare a launch argument for the parameter file
    default_params_file = str(
        Path(
            os.path.join(
                get_package_share_directory('holoocean_examples'),
                'config/key_teleop_config.yaml'
            )
        )
    )
    print('Using default parameter file: ', default_params_file)
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the node',
    )

    # Get the launch configuration for the parameter file
    params_file = LaunchConfiguration('params_file')

    holoocean_namespace = 'holoocean'

    twist_holoocean_node = launch_ros.actions.Node(
        name='twist_holoocean',
        package='holoocean_examples',
        namespace=holoocean_namespace,
        executable='twist_holoocean',
        output='screen',
        parameters=[params_file],
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

    return LaunchDescription([
        declare_params_file,
        holoocean_main_node,
        twist_holoocean_node,
    ])
