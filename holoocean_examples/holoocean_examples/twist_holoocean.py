#!/usr/bin/env python3
"""Twist → AgentCommand converter for HoloOcean agents.

Subscribes to geometry_msgs/Twist (e.g. from teleop_twist_keyboard) and
publishes holoocean_interfaces/AgentCommand.  Agent name and type are loaded
from the scenario JSON, mirroring the joy_holoocean approach.

Twist axis mapping
------------------
BlueROV2 / HoveringAUV:
  linear.x  → forward / backward (horizontal thrusters)
  linear.y  → strafe left / right
  linear.z  → ascend / descend (vertical thrusters)
  angular.z → yaw
  angular.y → pitch (scaled by yaw_scale)
  angular.x → roll  (scaled by yaw_scale)

CougUV / TorpedoAUV:
  linear.x  → thruster speed  (scaled to max_thrust)
  angular.y → pitch fins      (scaled to max_fin_delta)
  angular.z → yaw fins        (scaled to max_fin_delta)

SurfaceVessel:
  linear.x  → base speed for both thrusters
  angular.z → differential between left and right thrusters
"""

import json
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from holoocean_interfaces.msg import AgentCommand
from rclpy.node import Node


_MAX_THRUST_BLUEROV2 = 28.75
_MAX_THRUST_COUGUV = 1500.0
_MAX_THRUST_SURFACE = 1500.0
_MAX_FIN_DELTA = 0.1


class TwistToAgentCommand(Node):
    """Converts Twist messages to AgentCommand messages for HoloOcean."""

    def __init__(self):
        super().__init__('twist_holoocean')

        self.declare_parameter('relative_path', True)
        self.declare_parameter('scenario_path', '')
        self.declare_parameter('agents.default', 'auv0')

        def gs(p):
            return self.get_parameter(p).get_parameter_value().string_value

        scenario_path = gs('scenario_path')
        relative_path = self.get_parameter('relative_path').get_parameter_value().bool_value
        if relative_path:
            scenario_path = os.path.join(
                get_package_share_directory('holoocean_main'), scenario_path)
        if not os.path.isfile(scenario_path):
            raise ValueError(f'Scenario file not found: {scenario_path}')

        with open(scenario_path) as f:
            scenario = json.load(f)

        agents = scenario.get('agents', [])
        if not agents:
            raise ValueError('No agents found in scenario')

        default_name = gs('agents.default')
        agent = next((a for a in agents if a['agent_name'] == default_name), agents[0])
        self.agent_name = agent['agent_name']
        self.agent_type = agent['agent_type']

        self.get_logger().info(
            f'Twist converter: agent={self.agent_name}  type={self.agent_type}'
        )
        self.get_logger().info(
            'Waiting for Twist on cmd_vel. '
            'Run in a separate terminal: '
            'ros2 run teleop_twist_keyboard teleop_twist_keyboard '
            '--ros-args -r __ns:=/holoocean'
        )

        self.agent_pub = self.create_publisher(AgentCommand, 'command/agent', 10)
        self.create_subscription(Twist, 'cmd_vel', self._twist_cb, 10)

    def _twist_cb(self, msg: Twist):
        cmd = AgentCommand()
        cmd.header.frame_id = self.agent_name

        if self.agent_type in ('BlueROV2', 'HoveringAUV'):
            cmd.command = self._bluerov2(msg)
        elif self.agent_type in ('CougUV', 'TorpedoAUV'):
            cmd.command = self._couguv(msg)
        elif self.agent_type == 'SurfaceVessel':
            cmd.command = self._surface(msg)
        else:
            return

        self.agent_pub.publish(cmd)

    def _bluerov2(self, t: Twist) -> list[float]:
        m = _MAX_THRUST_BLUEROV2
        forward  = t.linear.x
        lateral  = t.linear.y
        vertical = t.linear.z
        yaw      = t.angular.z * 0.05
        pitch    = t.angular.y * 0.2
        roll     = t.angular.x * 0.05
        command = [
            (vertical + pitch + roll)  * m,
            (vertical + pitch - roll)  * m,
            (vertical - pitch - roll)  * m,
            (vertical - pitch + roll)  * m,
            (forward  + yaw + lateral) * m,
            (forward  - yaw - lateral) * m,
            (forward  - yaw + lateral) * m,
            (forward  + yaw - lateral) * m,
        ]
        return [max(-m, min(m, v)) for v in command]

    def _couguv(self, t: Twist) -> list[float]:
        pitch = t.angular.y
        yaw   = t.angular.z
        speed = t.linear.x
        return [
             pitch * _MAX_FIN_DELTA,
             yaw   * _MAX_FIN_DELTA,
            -pitch * _MAX_FIN_DELTA,
            -yaw   * _MAX_FIN_DELTA,
             speed * _MAX_THRUST_COUGUV,
        ]

    def _surface(self, t: Twist) -> list[float]:
        m = _MAX_THRUST_SURFACE
        fwd  = t.linear.x
        turn = t.angular.z
        return [
            (fwd + turn) * m,
            (fwd - turn) * m,
        ]


def main(args=None):
    rclpy.init(args=args)
    node = TwistToAgentCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
