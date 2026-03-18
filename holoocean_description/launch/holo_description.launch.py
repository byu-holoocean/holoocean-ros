# Copyright (c) 2026 BYU FRoSt Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    PathJoinSubstitution,
    Command,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition


def generate_launch_description():

    pkg_share = FindPackageShare(package="holoocean_description").find("holoocean_description")

    use_sim_time = LaunchConfiguration("use_sim_time")
    urdf_file = LaunchConfiguration("urdf_file")
    auv_ns = LaunchConfiguration("auv_ns")
    frame_prefix = PythonExpression(
        ["'", auv_ns, "/' if '", auv_ns, "' != '' else ''"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (HoloOcean) clock if true",
            ),
            DeclareLaunchArgument(
                "urdf_file",
                default_value="urdf/couguv_holoocean.urdf.xacro",
                description="URDF or Xacro file to load",
            ),
            DeclareLaunchArgument(
                "auv_ns",
                default_value="auv0",
                description=(
                    "Namespace for the AUV (e.g. auv0), used for namespacing topics and frames"
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {
                        # Use the xacro command to process the URDF file
                        "robot_description": ParameterValue(
                            Command(
                                ["xacro ", PathJoinSubstitution([pkg_share, urdf_file])]
                            ),
                            value_type=str,
                        ),
                        "use_sim_time": use_sim_time,
                        "frame_prefix": frame_prefix,
                    }
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=UnlessCondition(
                    use_sim_time
                ),  # HoloOcean publishes joint states
            ),
        ]
    )
