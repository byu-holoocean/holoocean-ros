#!/usr/bin/env python3
"""Joystick → AgentCommand converter with per-agent state management.

Each agent is represented by an AgentController that tracks its own state
(camera tilt, trim values, etc.) and knows how to build ROS commands from
raw joystick axes.  The active agent is selected via a dedicated face button
configured in the parameter file.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from holoocean_interfaces.msg import AgentCommand, SensorCommand


def _ax(axes: list[float], index: int) -> float:
    """Safely index into an axes list, returning 0.0 if out of range."""
    return axes[index] if index < len(axes) else 0.0


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(value, hi))


# ---------------------------------------------------------------------------
#  Per-agent controller
# ---------------------------------------------------------------------------

class AgentController:
    """Holds per-agent mutable state and generates ROS commands from joystick input."""

    CAMERA_SENSOR = 'CameraSensor'
    TRIM_STEP = 0.05
    MAX_TRIM = 1.0

    def __init__(self, name: str, agent_type: str,
                 camera_pitch_rate: float, camera_pitch_max: float):
        self.name = name
        self.type = agent_type
        self.camera_pitch_rate = camera_pitch_rate
        self.camera_pitch_max = camera_pitch_max

        # Mutable state
        self.camera_pitch: float = 0.0
        self.pitch_trim: float = 0.0
        self.roll_trim: float = 0.0

    def reset(self):
        """Reset all stateful values (call on disarm or simulation reset)."""
        self.camera_pitch = 0.0
        self.pitch_trim = 0.0
        self.roll_trim = 0.0

    def camera_command(self, axes: list[float],
                       cam_up_idx: int, cam_down_idx: int) -> SensorCommand | None:
        """Compute a SensorCommand from trigger axes.

        Trigger convention: rest = 1.0, fully pressed = -1.0.
        Returns None if neither trigger is meaningfully depressed.
        """
        up_raw   = _ax(axes, cam_up_idx)
        down_raw = _ax(axes, cam_down_idx)
        up   = (1.0 - up_raw)   / 2.0   # 0.0 at rest → 1.0 fully pressed
        down = (1.0 - down_raw) / 2.0

        if up <= 0.05 and down <= 0.05:
            return None

        self.camera_pitch += (up - down) * self.camera_pitch_rate
        self.camera_pitch = _clamp(self.camera_pitch,
                                   -self.camera_pitch_max,
                                    self.camera_pitch_max)
        cmd = SensorCommand()
        cmd.agent_name  = self.name
        cmd.sensor_name = self.CAMERA_SENSOR
        cmd.rotation    = [0.0, self.camera_pitch, 0.0]
        return cmd

    def agent_command(self, axes: list[float],
                      axis_map: dict[str, int]) -> tuple[AgentCommand, bool]:
        """Build an AgentCommand from joystick axes.

        Returns (AgentCommand, use_control_publisher).
        use_control_publisher=True means publish to command/control (FOSSEN),
        False means publish to command/agent.
        """
        def ax(key: str) -> float:
            return _ax(axes, axis_map[key])

        cmd = AgentCommand()
        cmd.header.frame_id = self.name
        use_control = False

        if self.type == 'bluerov':
            max_thrust = 28.75

            forward  = ax('vertical_left')
            vertical = ax('vertical_right')
            yaw      = ax('horizontal_right') * 0.05
            lateral  = ax('horizontal_left')

            self.pitch_trim = self._update_trim(self.pitch_trim, ax('dpad_vertical'))
            self.roll_trim  = self._update_trim(self.roll_trim,  ax('dpad_horizontal'))

            pitch = self.pitch_trim * 0.2
            roll  = self.roll_trim  * 0.05

            cmd.command = [
                (vertical + pitch + roll)  * max_thrust,
                (vertical + pitch - roll)  * max_thrust,
                (vertical - pitch - roll)  * max_thrust,
                (vertical - pitch + roll)  * max_thrust,
                (forward  + yaw + lateral) * max_thrust,
                (forward  - yaw - lateral) * max_thrust,
                (forward  - yaw + lateral) * max_thrust,
                (forward  + yaw - lateral) * max_thrust,
            ]

        elif self.type == 'surface_vessel':
            max_thrust = 1500
            cmd.command = [
                ax('vertical_left')  * max_thrust,
                ax('vertical_right') * max_thrust,
            ]

        elif self.type == 'torpedo_auv':
            max_thrust = 1500
            max_delta  = 0.1
            pitch = ax('vertical_left')
            yaw   = ax('horizontal_left')
            speed = ax('vertical_right')
            cmd.command = [
                 pitch * max_delta,
                 yaw   * max_delta,
                -pitch * max_delta,
                -yaw   * max_delta,
                 speed * max_thrust,
            ]
            use_control = True

        return cmd, use_control

    def _update_trim(self, trim: float, input_val: float) -> float:
        if input_val > 0.5:
            trim += self.TRIM_STEP
        elif input_val < -0.5:
            trim -= self.TRIM_STEP
        return _clamp(trim, -self.MAX_TRIM, self.MAX_TRIM)


# ---------------------------------------------------------------------------
#  ROS 2 node
# ---------------------------------------------------------------------------

class JoyToAgentCommand(Node):
    def __init__(self):
        super().__init__('joy_holoocean')

        # Publishers
        self.agent_pub   = self.create_publisher(AgentCommand, 'command/agent',   10)
        self.control_pub = self.create_publisher(AgentCommand, 'command/control', 10)
        self.sensor_pub  = self.create_publisher(SensorCommand, 'command/sensor', 10)

        self.reset_client = self.create_client(Trigger, 'reset')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('button.arm',    11)
        self.declare_parameter('button.disarm', 10)
        self.declare_parameter('button.reset',  12)

        self.declare_parameter('axis.vertical_left',    1)
        self.declare_parameter('axis.horizontal_left',  0)
        self.declare_parameter('axis.vertical_right',   3)
        self.declare_parameter('axis.horizontal_right', 2)
        self.declare_parameter('axis.camera_up',        5)
        self.declare_parameter('axis.camera_down',      4)
        self.declare_parameter('axis.dpad_vertical',    7)
        self.declare_parameter('axis.dpad_horizontal',  6)

        self.declare_parameter('camera.pitch_rate', 2.0)
        self.declare_parameter('camera.pitch_max',  40.0)

        # Agents are described by three parallel arrays.
        self.declare_parameter('agents.names',   ['auv0'])
        self.declare_parameter('agents.types',   ['bluerov'])
        self.declare_parameter('agents.buttons', [0])
        self.declare_parameter('agents.default', 'auv0')

        # ── Read parameters ──────────────────────────────────────────────────
        def gi(p): return self.get_parameter(p).get_parameter_value().integer_value
        def gf(p): return self.get_parameter(p).get_parameter_value().double_value
        def gs(p): return self.get_parameter(p).get_parameter_value().string_value

        self.btn_arm    = gi('button.arm')
        self.btn_disarm = gi('button.disarm')
        self.btn_reset  = gi('button.reset')

        self.axis_map: dict[str, int] = {
            'vertical_left':    gi('axis.vertical_left'),
            'horizontal_left':  gi('axis.horizontal_left'),
            'vertical_right':   gi('axis.vertical_right'),
            'horizontal_right': gi('axis.horizontal_right'),
            'camera_up':        gi('axis.camera_up'),
            'camera_down':      gi('axis.camera_down'),
            'dpad_vertical':    gi('axis.dpad_vertical'),
            'dpad_horizontal':  gi('axis.dpad_horizontal'),
        }

        pitch_rate = gf('camera.pitch_rate')
        pitch_max  = gf('camera.pitch_max')

        names   = list(self.get_parameter('agents.names').get_parameter_value().string_array_value)
        types   = list(self.get_parameter('agents.types').get_parameter_value().string_array_value)
        buttons = list(self.get_parameter('agents.buttons').get_parameter_value().integer_array_value)
        default = gs('agents.default')

        if not (len(names) == len(types) == len(buttons)):
            raise ValueError(
                'agents.names, agents.types, and agents.buttons must have the same length '
                f'(got {len(names)}, {len(types)}, {len(buttons)})'
            )

        # Build controllers
        self.controllers: dict[str, AgentController] = {}
        self.button_to_agent: dict[int, AgentController] = {}

        for name, atype, btn in zip(names, types, buttons):
            ctrl = AgentController(name, atype, pitch_rate, pitch_max)
            self.controllers[name] = ctrl
            self.button_to_agent[int(btn)] = ctrl
            self.get_logger().info(f'  agent [{name}]  type={atype}  button={btn}')

        if default not in self.controllers:
            default = names[0]
        self.active: AgentController = self.controllers[default]

        self.enabled = False

        self.create_subscription(Joy, 'joy', self._joy_cb, 10)
        self.get_logger().info(
            f'Joy node ready.  Default agent: {self.active.name}'
        )

    # ── Joy callback ────────────────────────────────────────────────────────

    def _btn(self, msg: Joy, index: int) -> bool:
        return len(msg.buttons) > index and bool(msg.buttons[index])

    def _joy_cb(self, msg: Joy):
        # Arm / disarm / reset
        if self._btn(msg, self.btn_arm):
            self.enabled = True
            self.get_logger().info('ENABLED')

        if self._btn(msg, self.btn_disarm):
            self.enabled = False
            self.active.reset()
            self.get_logger().info('DISABLED')

        if self._btn(msg, self.btn_reset):
            self.reset_client.call_async(Trigger.Request())
            self.active.reset()

        # Agent selection
        for btn, ctrl in self.button_to_agent.items():
            if self._btn(msg, btn):
                self.active = ctrl
                self.get_logger().info(f'Active agent → {ctrl.name} ({ctrl.type})')

        if not self.enabled:
            return

        axes = list(msg.axes)

        # Camera pitch (runs for all agent types)
        cam_cmd = self.active.camera_command(
            axes,
            self.axis_map['camera_up'],
            self.axis_map['camera_down'],
        )
        if cam_cmd is not None:
            self.sensor_pub.publish(cam_cmd)

        # Vehicle command
        cmd, use_control = self.active.agent_command(axes, self.axis_map)
        if use_control:
            self.control_pub.publish(cmd)
        else:
            self.agent_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToAgentCommand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
