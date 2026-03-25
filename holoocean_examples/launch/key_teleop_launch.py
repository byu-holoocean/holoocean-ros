## HoloOcean keyboard teleop launch file
#
# Starts the simulation and a Twist-to-AgentCommand converter node.
# Run teleop_twist_keyboard separately in its own terminal:
#
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
#       --ros-args -r __ns:=/holoocean

from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    print('Launching HoloOcean Keyboard Teleop')

    base = Path(get_package_share_directory('holoocean_examples'))
    params_file = base / 'config' / 'key_teleop_config.yaml'

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
        holoocean_main_node,
        twist_holoocean_node,
    ])
