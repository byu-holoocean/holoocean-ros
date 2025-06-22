import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from holoocean_interfaces.msg import AgentCommand, SensorCommand

class JoyToAgentCommand(Node):
    def __init__(self):
        super().__init__('joy_holoocean')

        self.publisher_ = self.create_publisher(AgentCommand, 'command/agent', 10)
        self.sensor_command_publisher_ = self.create_publisher(SensorCommand, 'command/sensor', 10)
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

        # Use these button indices as needed
        self.arm_button_index = 11  # e.g., A button
        self.disarm_button_index = 10  # e.g., B button

        self.vertical_axis_index = 3   # Right Joystick Vertical Axis 
        self.yaw_axis_index = 2     # Right Joystick Horizontal Axis
        self.forward_axis_index = 1     # Left Joystick Vertical Axis
        self.left_axis_index = 0     # Left Joystick Horizontal Axis

        # Camera Pitch axis
        self.up_camera_pitch_axis_index = 5     # Left throttle 
        self.down_camera_pitch_axis_index = 4   # Right throttle 

        # Trim Pitch and Roll
        self.pitch_axis_index = 7   # Vertical DPAD
        self.roll_axis_index = 6   # Horizontal DPAD

        self.enabled = False
        self.get_logger().info('Joy to AgentCommand node has started.')

    

    def joy_callback(self, msg: Joy):
        # Toggle enable/disable based on button press
        if len(msg.buttons) > self.arm_button_index and msg.buttons[self.arm_button_index]:
            self.enabled = True
            self.get_logger().info('AgentCommand publishing ENABLED')

        if len(msg.buttons) > self.disarm_button_index and msg.buttons[self.disarm_button_index]:
            self.enabled = False
            self.get_logger().info('AgentCommand publishing DISABLED')

        if not self.enabled:
            return

        agent_cmd = AgentCommand()

        
        # TODO parameterize this as holoocean vehicle
        agent_cmd.header.frame_id = 'auv0'

        # "[Vertical Front Starboard, Vertical Front Port, Vertical Back Port, Vertical Back Starboard, Angled Front Starboard, Angled Front Port, Angled Back Port, Angled Back Starboard]"
        max_thrust = 28.75
        
        # Map joystick inputs to thruster outputs (example for differential thrust logic)
        forward = msg.axes[self.forward_axis_index] if len(msg.axes) > self.forward_axis_index else 0.0
        vertical = msg.axes[self.vertical_axis_index] if len(msg.axes) > self.vertical_axis_index else 0.0
        yaw = msg.axes[self.yaw_axis_index] if len(msg.axes) > self.yaw_axis_index else 0.0
        left = msg.axes[self.left_axis_index] if len(msg.axes) > self.left_axis_index else 0.0
        pitch = msg.axes[self.pitch_axis_index] if len(msg.axes) > self.pitch_axis_index else 0.0
        roll = msg.axes[self.roll_axis_index] if len(msg.axes) > self.roll_axis_index else 0.0
        
        angular_scalar = 0.05    # Scale down yaw to give more fine control
        yaw *= angular_scalar
        pitch *= angular_scalar
        roll *= angular_scalar

        # Basic mixing logic for 8-thruster HoloOcean AUV
        agent_cmd.command = [
            (vertical - pitch + roll)* max_thrust,  # Vertical Front Starboard
            (vertical - pitch - roll)* max_thrust,  # Vertical Front Port
            (vertical + pitch - roll)* max_thrust,  # Vertical Back Port
            (vertical + pitch + roll)* max_thrust,  # Vertical Back Starboard
            # TODO fix so it doesnt go over max thrust
            (forward + yaw + left) * max_thrust,  # Angled Front Starboard
            (forward - yaw - left) * max_thrust,  # Angled Front Port
            (forward - yaw + left) * max_thrust,  # Angled Back Port
            (forward + yaw - left) * max_thrust   # Angled Back Starboard
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
