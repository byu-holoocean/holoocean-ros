import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from holoocean_interfaces.msg import AgentCommand #, SensorCommand
from enum import Enum


class AgentType(Enum):
    BLUEROV = 1
    SURFACE_VESSEL = 2


agent_keys = {
    'bluerov': AgentType.BLUEROV,
    'surface_vessel': AgentType.SURFACE_VESSEL
}


class JoyToAgentCommand(Node):
    def __init__(self):
        super().__init__('joy_holoocean')

        self.publisher_ = self.create_publisher(AgentCommand, 'command/agent', 10)
        # self.sensor_command_publisher_ = self.create_publisher(SensorCommand, 'command/sensor', 10)
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

        ## JOYSTICK PARAMS ##
        self.arm_button_index = 11
        self.disarm_button_index = 10

        self.vertical_axis_index = 3
        self.yaw_axis_index = 2
        self.forward_axis_index = 1
        self.left_axis_index = 0

        self.up_camera_pitch_axis_index = 5
        self.down_camera_pitch_axis_index = 4

        self.pitch_axis_index = 7
        self.roll_axis_index = 6

        ## BLUE ROV PARAMS ##
        self.pitch_trim = 0.0
        self.roll_trim = 0.0
        self.trim_step = 0.05
        self.max_trim = 1.0

        agent_param = self.declare_parameter("agent", "bluerov").value
        self.agent = agent_keys.get(agent_param.lower(), AgentType.BLUEROV)

        self.enabled = False
        self.get_logger().info('Joy to AgentCommand node has started.')

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))

    def get_axis_value(self, msg, index):
        return msg.axes[index] if len(msg.axes) > index else 0.0

    def update_trim(self, trim_value, input_value):
        if input_value > 0.5:
            trim_value += self.trim_step
        elif input_value < -0.5:
            trim_value -= self.trim_step
        return self.clamp(trim_value, -self.max_trim, self.max_trim)

    def joy_callback(self, msg: Joy):
        if len(msg.buttons) > self.arm_button_index and msg.buttons[self.arm_button_index]:
            self.enabled = True
            self.get_logger().info('AgentCommand publishing ENABLED')

        if len(msg.buttons) > self.disarm_button_index and msg.buttons[self.disarm_button_index]:
            self.enabled = False
            self.get_logger().info('AgentCommand publishing DISABLED')

        if not self.enabled:
            return

        agent_cmd = AgentCommand()
        agent_cmd.header.frame_id = 'auv0'  # TODO: get from parameter

        if self.agent == AgentType.BLUEROV:
            max_thrust = 28.75

            forward = self.get_axis_value(msg, self.forward_axis_index)
            vertical = self.get_axis_value(msg, self.vertical_axis_index)
            yaw = self.get_axis_value(msg, self.yaw_axis_index)
            left = self.get_axis_value(msg, self.left_axis_index)

            pitch_input = self.get_axis_value(msg, self.pitch_axis_index)
            roll_input = self.get_axis_value(msg, self.roll_axis_index)

            self.pitch_trim = self.update_trim(self.pitch_trim, pitch_input)
            self.roll_trim = self.update_trim(self.roll_trim, roll_input)

            angular_scalar = 0.05
            pitch_scalar = 0.2

            yaw *= angular_scalar
            pitch = self.pitch_trim * pitch_scalar
            roll = self.roll_trim * angular_scalar

            agent_cmd.command = [
                (vertical + pitch + roll) * max_thrust,
                (vertical + pitch - roll) * max_thrust,
                (vertical - pitch - roll) * max_thrust,
                (vertical - pitch + roll) * max_thrust,
                (forward + yaw + left) * max_thrust,
                (forward - yaw - left) * max_thrust,
                (forward - yaw + left) * max_thrust,
                (forward + yaw - left) * max_thrust
            ]

        elif self.agent == AgentType.SURFACE_VESSEL:
            left_thrust = self.get_axis_value(msg, self.forward_axis_index)
            right_thrust = self.get_axis_value(msg, self.vertical_axis_index)

            max_thrust = 1500
            agent_cmd.command = [
                left_thrust * max_thrust,
                right_thrust * max_thrust
            ]

        self.publisher_.publish(agent_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToAgentCommand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
