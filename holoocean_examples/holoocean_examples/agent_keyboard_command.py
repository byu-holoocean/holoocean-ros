import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from holoocean_interfaces.msg import AgentCommand
from pynput import keyboard
import numpy as np
import threading
import time

class AgentKeyboard(Node):
    def __init__(self):
        super().__init__('agent_keyboard')

        self.publisher = self.create_publisher(AgentCommand, 'holoocean/command/agent', 10)
        self.vehicle_name = 'auv0'
        self.force = 30
        self.pressed_keys = set()

        # Start the keyboard listener in a thread
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.daemon = True
        listener.start()

        # Timer to publish at fixed rate
        self.timer = self.create_timer(0.1, self.publish_command)  # 10 Hz

    def on_press(self, key):
        if hasattr(key, 'char') and key.char is not None:
            self.pressed_keys.add(key.char)

    def on_release(self, key):
        if hasattr(key, 'char') and key.char in self.pressed_keys:
            self.pressed_keys.remove(key.char)

    def parse_keys(self, keys, val):
        command = np.zeros(5)

        if 't' in keys:  # forward thrust
            command[4] += val
        if 'g' in keys:  # backward thrust
            command[4] -= val
        if 'f' in keys:  # roll CCW
            command[0:4] -= val
        if 'h' in keys:  # roll CW
            command[0:4] += val
        if 'i' in keys:  # pitch up
            command[0] -= val
            command[2] += val
        if 'k' in keys:  # pitch down
            command[0] += val
            command[2] -= val
        if 'j' in keys:  # yaw left
            command[1] -= val
            command[3] += val
        if 'l' in keys:  # yaw right
            command[1] += val
            command[3] -= val

        return command.tolist()

    def publish_command(self):
        msg = AgentCommand()
        msg.header = Header()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.vehicle_name
        msg.command = self.parse_keys(self.pressed_keys, self.force)
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.command}")


def main(args=None):
    rclpy.init(args=args)
    node = AgentKeyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
