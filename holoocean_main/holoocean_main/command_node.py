from holoocean_main.holoocean_interface import HolooceanInterface
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
import numpy as np


class CommandExample(Node):

    def __init__(self):
        super().__init__('command_node')

        self.declare_parameter('params_file', '')
        
        file_path = self.get_parameter('params_file').get_parameter_value().string_value
        interface = HolooceanInterface(file_path, init=False)

        self.time_warp = interface.get_time_warp()

        self.depth_publisher = self.create_publisher(Float64, 'depth', 10)
        self.heading_publisher = self.create_publisher(Float64, 'heading', 10)
        self.speed_publisher = self.create_publisher(Float64, 'speed', 10)
        
        # self.use_random = self.get_parameter('random').get_parameter_value().bool_value
        self.use_random = False
        self.sequence_index = 0

        # Predefined sequence of headings, speeds, and depths


        self.deep_predefined_sequence = [
            {'depth': 270.0, 'heading': 90.0, 'speed': 500.0},
            {'depth': 250.1, 'heading': 80.0, 'speed': 500.0},
            {'depth': 280.0, 'heading': 60.0, 'speed': 500.0},
            # {'depth': 20.0, 'heading': 90.0, 'speed': 2.0},
            # {'depth': 12.0, 'heading': 90.0, 'speed': 2.0},
            # {'depth': 18.0, 'heading': 90.0, 'speed': 2.0},
            # {'depth': 22.0, 'heading': 90.0, 'speed': 2.0},
            # {'depth': 8.0, 'heading': 90.0, 'speed': 2.0},
            # {'depth': 0.1, 'heading': 90.0, 'speed': 2.0},
            # {'depth': 14.0, 'heading': 90.0, 'speed': 2.0},
            # {'depth': 19.0, 'heading': 90.0, 'speed': 2.0},
            # {'depth': 25.0, 'heading': 90.0, 'speed': 2.0},
        ]

        self.predefined_sequence = [
            {'depth': 2.0, 'heading': 90.0, 'speed': 2.0},
            {'depth': 0.1, 'heading': 90.0, 'speed': 2.0},
            {'depth': 5.0, 'heading': 90.0, 'speed': 2.0},
            # {'depth': 20.0, 'heading': 90.0, 'speed': 2.0},
            # {'depth': 12.0, 'heading': 90.0, 'speed': 2.0},
            # {'depth': 18.0, 'heading': 90.0, 'speed': 2.0},
            # {'depth': 22.0, 'heading': 90.0, 'speed': 2.0},
            # {'depth': 8.0, 'heading': 90.0, 'speed': 2.0},
            # {'depth': 0.1, 'heading': 90.0, 'speed': 2.0},
            # {'depth': 14.0, 'heading': 90.0, 'speed': 2.0},
            # {'depth': 19.0, 'heading': 90.0, 'speed': 2.0},
            # {'depth': 25.0, 'heading': 90.0, 'speed': 2.0},
        ]

        #Setup timer to continue publishing depth heading
        timer_period = 150.0 / self.time_warp  # seconds
        timer_publish_period = 0.5 / self.time_warp  # seconds

        if self.use_random:
            self.randomize_callback()
            self.timer_setpoint = self.create_timer(timer_period, self.randomize_callback)
        else:
            self.sequence_callback()
            self.timer_setpoint = self.create_timer(timer_period, self.sequence_callback)
            
        self.timer_publish = self.create_timer(timer_publish_period, self.publish_callback)


    def sequence_callback(self):
        sequence = self.predefined_sequence
        if self.sequence_index < len(sequence):
            self.depth = sequence[self.sequence_index]['depth']
            self.heading = sequence[self.sequence_index]['heading']
            self.speed = sequence[self.sequence_index]['speed']
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
        depth_msg = Float64()
        depth_msg.data = self.depth
        self.depth_publisher.publish(depth_msg)
        heading_msg = Float64()
        heading_msg.data = self.heading
        self.heading_publisher.publish(heading_msg)
        speed_msg = Float64()
        speed_msg.data = self.speed
        self.speed_publisher.publish(speed_msg)


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