from holoocean_interfaces.msg import AgentCommand #, SensorCommand
from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import numpy as np

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

class CommandExample(Node):

    def __init__(self):
        super().__init__('waypoint_follower')

        self.waypoint_publisher = self.create_publisher(AgentCommand, 'command/agent', 10)
        self.gps_subscriber = self.create_subscription(Odometry, '/holoocean/asv0/GPSSensor', self.gps_callback, 10)

        self.waypoint_threshold = 3.0  # how close in meters the vessel needs to be to consider the waypoint reached
        self.waypoint_idx = 0
        self.waypoint_locations = np.array([[ 25, 25],
                                            [-25, 25],
                                            [-25,-25],
                                            [ 25,-25]])
        
        self.current_position = np.array([0.0, 0.0]) # Only tracking XY position for surface vessel

        # Simulated clock-based timing using ROS time
        self.sim_clock = self.get_clock()

        # Immediately publish the first setpoint
        self.last_publish_time = self.sim_clock.now()
        self.publish_interval = Duration(seconds=0.5)

        self.create_timer(0.1, self.timer_callback)

        self.marker_pub = self.create_publisher(Marker, 'debug/points', 10)
        
    def gps_callback(self, msg):
        self.current_position = np.array([msg.pose.pose.position.x,
                                          msg.pose.pose.position.y])
    def timer_callback(self):
        distance = np.linalg.norm(self.current_position - self.waypoint_locations[self.waypoint_idx])
        # self.get_logger().info(f"Distance to waypoint: {distance:.2f} meters")
        if np.linalg.norm(self.current_position - self.waypoint_locations[self.waypoint_idx]) < self.waypoint_threshold:
            self.waypoint_idx = (self.waypoint_idx + 1) % len(self.waypoint_locations)
            self.get_logger().info(f"Reached waypoint, moving to waypoint {self.waypoint_idx}")
        now = self.sim_clock.now()
        if now - self.last_publish_time >= self.publish_interval:
            self.publish_callback()
            self.last_publish_time = now

    def publish_callback(self):
        base_msg = AgentCommand()
        base_msg.header.stamp = self.sim_clock.now().to_msg()
        # TODO parameterize this as holoocean vehicle
        base_msg.header.frame_id = 'asv0'

        base_msg.command = self.waypoint_locations[self.waypoint_idx].tolist()
        self.waypoint_publisher.publish(base_msg)

        self.publish_waypoint_markers(self.waypoint_locations) # If this is in the init function, the markers do not show up as holoocean takes time to load

    def publish_waypoint_markers(self, waypoint_locations):
        """
        Publishes a list of [x, y, z] coordinates as a single ROS2 Marker.
        waypoint_locations: List of lists or Nx3 numpy array
        """
        marker = Marker()
        
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "Waypoints"
        marker.id = 0
        
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        # Scale in meters
        marker.scale.x = 0.1  # Diameter of the points
        marker.scale.y = 0.1
        
        # Color (Defaulting to Red: [255, 0, 0] -> 1.0, 0.0, 0.0)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        # Lifetime (0 = persistent)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        # Convert waypoints to geometry_msgs/Point objects
        marker.points = []
        for loc in waypoint_locations:
            p = Point()
            p.x = float(loc[0])
            p.y = float(loc[1])
            p.z = 0.5
            marker.points.append(p)
            
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)

    command_node = CommandExample()

    rclpy.spin(command_node)

    command_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
