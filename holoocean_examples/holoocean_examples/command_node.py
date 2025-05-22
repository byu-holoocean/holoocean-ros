from holoocean_interfaces.msg import DesiredCommand
import rclpy
from rclpy.node import Node
from rclpy.clock import ClockType
from rclpy.time import Time
from rclpy.duration import Duration

from std_msgs.msg import Float64
import numpy as np

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

# Define a QoS profile that allows late-joining subscribers to receive last message
qos_profile_transient_local = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)


class CommandExample(Node):

    def __init__(self):
        super().__init__('command_node')

        # TODO make these transient local qos for publishing
        self.depth_publisher = self.create_publisher(DesiredCommand, 'depth', qos_profile_transient_local)
        self.heading_publisher = self.create_publisher(DesiredCommand, 'heading', qos_profile_transient_local)
        self.speed_publisher = self.create_publisher(DesiredCommand, 'speed', qos_profile_transient_local)


        self.declare_parameter('random', False)
        self.declare_parameter('deep', False)

        self.use_random = self.get_parameter('random').get_parameter_value().bool_value
        self.use_deep = self.get_parameter('deep').get_parameter_value().bool_value

        self.sequence_index = 0

        self.deep_predefined_sequence = [
            {'depth': 295.0, 'heading': 90.0, 'speed': 1200.0},
            {'depth': 290.0, 'heading': 70.0, 'speed': 1200.0},
            {'depth': 285.0, 'heading': 50.0, 'speed': 1200.0},
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
        ]

        # Simulated clock-based timing using ROS time
        self.sim_clock = self.get_clock()

        # Immediately publish the first setpoint
        self.new_setpoint()
        self.setpoint_interval = Duration(seconds=150.0)
        self.last_publish_time = self.sim_clock.now()
        self.publish_interval = Duration(seconds=0.5)

        self.create_timer(0.1, self.timer_callback)

    def new_setpoint(self):
        if self.use_random:
            self.randomize_callback()
        else:
            self.sequence_callback()
        self.last_setpoint_time = self.sim_clock.now()
        self.enabled = True
    
    def timer_callback(self):
        now = self.sim_clock.now()
        # self.get_logger().info(f'Timer callback triggered  {now.nanoseconds} ns')

        if now - self.last_setpoint_time >= self.setpoint_interval:
            self.new_setpoint()
        # else:
        #     self.get_logger().info(f'Setpoint not yet ready: {now.nanoseconds} ns')

        if now - self.last_publish_time >= self.publish_interval and self.enabled:
            self.publish_callback()
            self.last_publish_time = now
            # self.get_logger().info(f'set new publish time: {self.last_publish_time.nanoseconds} ns')
        # else:
        #     self.get_logger().info(f'Publish not yet ready: {now.nanoseconds} ns, {self.last_publish_time.nanoseconds} ns, enabled: {self.enabled}')


    def sequence_callback(self):
        sequence = self.deep_predefined_sequence if self.use_deep else self.predefined_sequence
        if self.sequence_index >= len(sequence):
            self.sequence_index = 0

        self.depth = sequence[self.sequence_index]['depth']
        self.heading = sequence[self.sequence_index]['heading']
        self.speed = sequence[self.sequence_index]['speed']
        self.sequence_index += 1
        self.get_logger().info(f'New Setpoint: {self.depth}, Heading: {self.heading}, Speed: {self.speed}')

    def randomize_callback(self):
        self.depth = float(np.random.randint(0, 51))
        self.heading = float(np.random.randint(0, 360))
        self.speed = float(np.random.randint(1000, 1525))
        self.get_logger().info(f'New Setpoint: {self.depth}, Heading: {self.heading}, Speed: {self.speed}')

    def publish_callback(self):
        # self.get_logger().info(f'Publishing Setpoint: {self.depth}, Heading: {self.heading}, Speed: {self.speed}')
        base_msg = DesiredCommand()
        base_msg.header.stamp = self.sim_clock.now().to_msg()
        # TODO parameterize this as holoocean vehicle
        base_msg.header.frame_id = 'auv0'

        # TODO does base msg need to be copied?
        depth_msg = base_msg
        depth_msg.data = self.depth
        self.depth_publisher.publish(depth_msg)
        heading_msg = base_msg
        heading_msg.data = self.heading
        self.heading_publisher.publish(heading_msg)
        speed_msg = base_msg
        speed_msg.data = self.speed
        self.speed_publisher.publish(speed_msg)


def main(args=None):
    rclpy.init(args=args)

    command_node = CommandExample()

    rclpy.spin(command_node)

    command_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
