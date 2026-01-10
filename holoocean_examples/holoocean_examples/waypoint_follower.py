import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from holoocean_interfaces.msg import AgentCommand #, SensorCommand
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')
        # Waypoint parameters
        self.declare_parameter('waypoint_threshold', 3.0)
        self.declare_parameter('waypoint_file', 'config/sv_waypoints.yaml')
        self.declare_parameter('relative_path', True)
        self.declare_parameter('vehicle_name', 'asv0')
        self.declare_parameter('loop_waypoints', True)
        self.declare_parameter('marker_scale', 0.2)
        self.declare_parameter('marker_frame', 'world')

        self.marker_frame = self.get_parameter('marker_frame').value
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value  # how close in meters the vessel needs to be to consider the waypoint reached
        self.vehicle_name = self.get_parameter('vehicle_name').value  # Default vehicle name
        self.loop_waypoints = self.get_parameter('loop_waypoints').value
        self.marker_scale = self.get_parameter('marker_scale').value
        self.waypoint_locations = self.get_waypoints_from_file(
            self.get_parameter('waypoint_file').value,
            self.get_parameter('relative_path').value
        )
        if len(self.waypoint_locations) == 0:
            raise RuntimeError("Waypoint file contains no waypoints")
        self.waypoint_idx = 0

        self.waypoint_publisher = self.create_publisher(AgentCommand, 'command/agent', 10)
        self.gps_subscriber = self.create_subscription(Odometry, f'/holoocean/{self.vehicle_name}/GPSSensor', self.gps_callback, 10)
        
        self.current_position = np.array([0.0, 0.0]) # Only tracking XY position for surface vessel

        # Simulated clock-based timing using ROS time
        self.sim_clock = self.get_clock()

        self.last_publish_time = self.sim_clock.now()
        self.publish_interval = Duration(seconds=0.5)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.marker_pub = self.create_publisher(Marker, 'debug/points', 10)
        
    def get_waypoints_from_file(self, waypoint_file, relative_path):
        if relative_path:
            package_share_directory = Path(get_package_share_directory('holoocean_examples'))
            waypoint_file = os.path.join(package_share_directory, waypoint_file)
        with open(waypoint_file, 'r') as f:
            waypoint_data = yaml.safe_load(f)
        waypoints = np.array(waypoint_data['waypoints'])

        self.get_logger().info(f"Loaded {len(waypoints)} waypoints from {waypoint_file}")
        return waypoints
    
    def gps_callback(self, msg):
        self.current_position = np.array([msg.pose.pose.position.x,
                                          msg.pose.pose.position.y])
    
    def reset_waypoints(self):
        self.waypoint_idx = 0
        self.get_logger().info("Waypoints reset, starting from the first waypoint.")
        # self.timer.reset() needs to go here if this is modified to be called from service 

    def timer_callback(self):
        distance = np.linalg.norm(self.current_position - self.waypoint_locations[self.waypoint_idx])
        # self.get_logger().info(f"Distance to waypoint: {distance:.2f} meters")
        if np.linalg.norm(self.current_position - self.waypoint_locations[self.waypoint_idx]) < self.waypoint_threshold:
            self.get_logger().info(f"Reached waypoint {self.waypoint_idx}")
            if self.waypoint_idx < len(self.waypoint_locations) -1:
                self.waypoint_idx += 1
                self.get_logger().info(f"Moving to waypoint {self.waypoint_idx}")
            else:
                if self.loop_waypoints:
                    self.reset_waypoints()        
                else:
                    self.get_logger().info("Final waypoint reached, holding position.")
                    self.timer.cancel()  # Do not publish new waypoints if final waypoint is reached
        now = self.sim_clock.now()
        if now - self.last_publish_time >= self.publish_interval:
            self.publish_callback()
            self.last_publish_time = now

    def publish_callback(self):
        base_msg = AgentCommand()
        base_msg.header.stamp = self.sim_clock.now().to_msg()
        # TODO parameterize this as holoocean vehicle
        base_msg.header.frame_id = self.vehicle_name

        base_msg.command = self.waypoint_locations[self.waypoint_idx].tolist()
        self.waypoint_publisher.publish(base_msg)

        self.publish_waypoint_markers(self.waypoint_locations) # If this is in the init function, the markers do not show up as holoocean takes time to load

    def publish_waypoint_markers(self, waypoint_locations):
        """
        Publishes a list of [x, y, z] coordinates as a single ROS2 Marker.
        waypoint_locations: List of lists or Nx3 numpy array
        """
        marker = Marker()
        
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "Waypoints"
        marker.id = 0
        
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        # Scale in meters
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale  
        # Lifetime (slightly longer than publish interval)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 600_000_000
        
        # Convert waypoints to geometry_msgs/Point objects
        marker.colors = []
        marker.points = []

        for i, loc in enumerate(waypoint_locations):
            p = Point()
            p.x = float(loc[0])
            p.y = float(loc[1])
            p.z = 0.5
            marker.points.append(p)

            # Color logic
            if i < self.waypoint_idx:
                # Completed waypoint -> Green
                c = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            elif i == self.waypoint_idx:
                # Active waypoint -> Yellow
                c = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            else:
                # Future waypoint -> Red
                c = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

            marker.colors.append(c)

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)

    command_node = WaypointFollower()

    rclpy.spin(command_node)

    command_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
