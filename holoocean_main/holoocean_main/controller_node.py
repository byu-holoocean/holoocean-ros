import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, Vector3Stamped
from holoocean_interfaces.msg import UCommand
import numpy as np

from pathlib import Path
import json

class ControllerExample(Node):

    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('params_file', '')
        
        file_path = Path(self.get_parameter('params_file').get_parameter_value().string_value)

        scenario = None

        with file_path.open() as params_file:
            scenario = json.load(params_file)
        
        self.time_warp = 1.0
        if "frames_per_sec" in scenario:
            self.time_warp = scenario["frames_per_sec"]/ scenario["ticks_per_sec"]

        print("Time Warp:", self.time_warp)

        ############ HSD subscribers: ############
        self.depth = 0.0
        self.heading = 0.0
        self.speed = 0.0

        self.depth_sub = self.create_subscription(
            Float64,
            'depth',
            self.depth_callback,
            10
        )
        
        self.heading_sub = self.create_subscription(
            Float64,
            'heading',
            self.heading_callback,
            10
        )

        self.speed_sub = self.create_subscription(
            Float64,
            'speed',
            self.speed_callback,
            10
        )

        ############ Data Subscriber ############

        self.vel_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            'DVLSensorVelocity',
            self.vel_callback,
            10)

        self.rotation_sub = self.create_subscription(
            Vector3Stamped,
            'RotationSensor',
            self.rotation_callback,
            10)

        self.depth_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'DepthSensor',
            self.depth_callback,
            10)
        
        ######## Control Surfaces Publishers #########

        self.publisher = self.self.create_publisher(UCommand, 'u_command', 10)

        ######## Control loop timer ###############

        if self.time_warp > 0:
            timer_period =   0.1 / self.time_warp  # seconds
            timer_publish_period = 0.5 / self.time_warp  # seconds
            self.timer_control = self.create_timer(timer_period, self.control_loop)
        else:
            ValueError("frames_per_sec cannot be 0 for time warping. Set a value > 0 ")

    def depth_callback(self, msg):
        self.depth = msg.data

    def heading_callback(self, msg):
        self.heading = msg.data

    def speed_callback(self, msg):
        self.speed = msg.data


def main(args=None):
    rclpy.init(args=args)

    node = ControllerExample()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()