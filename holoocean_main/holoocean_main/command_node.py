import rclpy
from rclpy.node import Node

from holoocean_interfaces.msg import HSD
import numpy as np

from pathlib import Path
import json

class CommandExample(Node):

    def __init__(self):
        super().__init__('command_node')

        self.declare_parameter('params_file', '')
        
        file_path = Path(self.get_parameter('params_file').get_parameter_value().string_value)

        scenario = None

        with file_path.open() as params_file:
            scenario = json.load(params_file)
        
        self.time_warp = 1.0
        if "frames_per_sec" in scenario:
            self.time_warp = scenario["frames_per_sec"]/ scenario["ticks_per_sec"]

        print("Time Warp:", self.time_warp)

        self.publisher_ = self.create_publisher(HSD, 'desiredHSD', 10)
        
        self.use_random = False
        self.sequence_index = 0

        # Predefined sequence of headings, speeds, and depths
        self.predefined_sequence = [
            {'depth': 10.0, 'heading': 90.0, 'speed': 1200.0},
            {'depth': 0.1, 'heading': -60.0, 'speed': 1150.0},
            {'depth': 15.0, 'heading': 180.0, 'speed': 1300.0},
            {'depth': 20.0, 'heading': -90.0, 'speed': 1400.0},
            {'depth': 12.0, 'heading': 45.0, 'speed': 1250.0},
            {'depth': 18.0, 'heading': 135.0, 'speed': 1350.0},
            {'depth': 22.0, 'heading': -125.0, 'speed': 1450.0},
            {'depth': 8.0, 'heading': -45.0, 'speed': 1150.0},
            {'depth': 0.1, 'heading': -60.0, 'speed': 1150.0},
            {'depth': 14.0, 'heading': -90.0, 'speed': 1250.0},
            {'depth': 19.0, 'heading': 180.0, 'speed': 1350.0},
            {'depth': 25.0, 'heading': 90.0, 'speed': 1500.0},
        ]

        #Setup timer to continue publishing depth heading
        if self.time_warp > 0:
            timer_period = 60.0 / self.time_warp  # seconds
            timer_publish_period = 0.5 / self.time_warp  # seconds
        else:
            ValueError("frames_per_sec cannot be 0 for time warping. Set a value > 0 ")

        if self.use_random:
            self.randomize_callback()
            self.timer_setpoint = self.create_timer(timer_period, self.randomize_callback)
        else:
            self.sequence_callback()
            self.timer_setpoint = self.create_timer(timer_period, self.sequence_callback)
            
        self.timer_publish = self.create_timer(timer_publish_period, self.publish_callback)


    def sequence_callback(self):
        if self.sequence_index < len(self.predefined_sequence):
            self.depth = self.predefined_sequence[self.sequence_index]['depth']
            self.heading = self.predefined_sequence[self.sequence_index]['heading']
            self.speed = self.predefined_sequence[self.sequence_index]['speed']
            self.sequence_index += 1
        else:
            self.sequence_index = 0  # Restart the sequence from the beginning
        self.get_logger().info(f'New Setpoint: {self.depth}, Heading: {self.heading}, Speed: {self.speed}')

    def randomize_callback(self):
        self.depth = float(np.random.randint(0, 51))
        self.heading = float(np.random.randint(0, 360))
        self.speed = float(np.random.randint(1000, 1525))
        self.get_logger().info(f'New Setpoint: {self.depth}, Heading: {self.heading}, Speed: {self.speed}')

    def publish_callback(self):
        msg = HSD()
        msg.depth = self.depth
        msg.heading = self.heading
        msg.speed = self.speed
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    command_node = CommandExample()

    rclpy.spin(command_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    command_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()