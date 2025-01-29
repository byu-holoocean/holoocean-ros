# from holoocean_main.holoocean_interface import HolooceanInterface
# import rclpy
# from rclpy.node import Node

# from std_msgs.msg import Float64
# from geometry_msgs.msg import PoseWithCovarianceStamped
# import numpy as np

# from loguru import logger

# class VehicleInterfaceNode(Node):
#     def __init__(self):
#         super().__init__("vehicle_interface_node")
#         self.get_logger().info(f"Vehicle interface node is up.")
        
#         self.declare_parameter('params_file', '')
        
#         file_path = self.get_parameter('params_file').get_parameter_value().string_value
#         interface = HolooceanInterface(file_path, init=False)
        
#         self.time_warp = interface.get_time_warp()
        
        
#         # Publishers for depth, heading, and speed
#         self.depth_publisher = self.create_publisher(Float64, "depth", 10)
#         self.heading_publisher = self.create_publisher(Float64, "heading", 10)
#         self.speed_publisher = self.create_publisher(Float64, "speed", 10)
        
#         # Waypoint list
#         self.waypoints = np.array([[2, 2, 1], [-2, 2, 2], [-2, -2, 0.1], [2, -2, 5]])
#         self.current_waypoint_index = 0
        
#         # Subscribe to state estimation topic from RK45 dead reacking state estimator
#         self.state_subscription = self.create_subscription(
#             PoseWithCovarianceStamped,
#             'dead_reckon',
#             self.state_callback,
#             10
#         )
        
#         # Current position (assumed to be updated from sensor feedback)
#         self.current_position = np.array([0.0, 0.0, 0.0])
#         # self.current_position = np.array([,,])

#         # Timer to publish commands
#         # timer_publish_period = 150 / self.time_warp  # seconds
#         # self.timer_publish_period = 15 / self.time_warp  # seconds
        
#         # self.timer = self.create_timer(self.timer_publish_period, self.publish_hsd_command)
        
#         self.timer_publish_period = 10 / self.time_warp  # Adjust this as needed
#         self.timer = self.create_timer(self.timer_publish_period, self.publish_hsd_command)

#     def state_callback(self, msg):
#         """Update the current position from state estimation from world model simulation."""
#         self.current_position = np.array([
#             msg.pose.pose.position.x,
#             msg.pose.pose.position.y,
#             msg.pose.pose.position.z
#         ])
            
#         self.get_logger().info(f"Updated position from simulation: {self.current_position}")

#     def publish_hsd_command(self):
        
#         # Get the current waypoint
#         waypoint = self.waypoints[self.current_waypoint_index]
#         self.get_logger().info(f"this is current waypoint: {waypoint}")     
        
#         # Calculate heading and distance
#         direction = waypoint[:2] - self.current_position[:2]  # x, y difference
#         distance = np.linalg.norm(direction)
        
#         if distance > 1e-1:  # Check if waypoint is reached
#             heading = np.degrees(np.arctan2(direction[1], direction[0])) % 360
#             depth = waypoint[2]  # Negative depth as per HoloOcean convention
#             speed = min(distance * 10, 2)  # Scaled speed, cap at 500
            
#             # Publish commands
#             self.publish_depth(depth)
#             self.publish_heading(heading)
#             self.publish_speed(speed)
#             self.get_logger().info(f"Published -- Depth: {depth}, heading: {heading} and speed: {speed}, Distance to waypoint: {distance}")
#         else:
#             # Move to next waypoint if close enough
#             self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
#             self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}, moving to next!")



#     def publish_depth(self, depth):
#         depth_msg = Float64()
#         depth_msg.data = float(depth)
#         self.depth_publisher.publish(depth_msg)
#         # self.get_logger().info(f"Published Depth: {depth}")

#     def publish_heading(self, heading):
#         heading_msg = Float64()
#         heading_msg.data = float(heading)
#         self.heading_publisher.publish(heading_msg)
#         # self.get_logger().info(f"Published Heading: {heading}")

#     def publish_speed(self, speed):
#         speed_msg = Float64()
#         speed_msg.data = float(speed)
#         self.speed_publisher.publish(speed_msg)
#         # self.get_logger().info(f"Published Speed: {speed}")


# def main(args=None):
#     rclpy.init(args=args)
#     vehicle_interface_node = VehicleInterfaceNode()
#     rclpy.spin(vehicle_interface_node)

#     # Shutdown
#     vehicle_interface_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()



from holoocean_main.holoocean_interface import HolooceanInterface
import rclpy
from rclpy.node import Node
import os
import time

from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import math
from loguru import logger

class VehicleInterfaceNode(Node):
    def __init__(self):
        super().__init__("vehicle_interface_node")
        self.get_logger().info("Vehicle interface node is up.")

        # Load parameters
        self.declare_parameter('params_file', '')
        file_path = self.get_parameter('params_file').get_parameter_value().string_value
        self.interface = HolooceanInterface(file_path, init=False)
        
        self.env_ready = False  # Track if the environment is ready
        
        self.first_command_sent = False # Define this attribute to track first command
        
        # Subscribe to environment readiness
        self.env_ready_subscription = self.create_subscription(
            Bool,
            'env_ready',
            self.env_ready_callback,
            10
        )
        
        self.time_warp = self.interface.get_time_warp()

        # Publishers for depth, heading, and speed
        self.depth_publisher = self.create_publisher(Float64, "depth", 10)
        self.heading_publisher = self.create_publisher(Float64, "heading", 10)
        self.speed_publisher = self.create_publisher(Float64, "speed", 10)

        # Waypoint list (will replace with waypoints from the path planner in future)
        # self.waypoints = np.array([[25, 25, 5.2], [-25, 25, 7.2], [-25, -25, 8.2], [25, -25, 7.2]])
        self.waypoints = np.array([[15, 15, 5], [-15, 15, 5], [-15, -15, 5], [15, -15, 5]])
        self.current_waypoint_index = 0

        # Subscribe to state estimation topic from RK45 dead reckoning state estimator
        self.state_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'dead_reckon',
            self.state_callback,
            10
        )

        # Initialize vehicle state
        self.current_position = np.array([0.0, 0.0, -5.0])
        self.current_heading = 0.0  # Assume initial heading
        self.distance_threshold =  1.6 * 10 # Optimal distance threshold 16.0 from fine tuning        

        self.rpm = 700  # Optimized RPM value
        
        
        # Timer to continuously publish HSD commands
        self.timer_publish_period = 10 / self.time_warp
        self.timer = self.create_timer(self.timer_publish_period, self.publish_hsd_command)
        
        # Log file setup
        log_file_path = os.path.expanduser("vehicle_trajectory.log")
        self.log_file = open(log_file_path, "w")
        self.log_file.write("Time,X,Y,Z,WaypointX,WaypointY,WaypointZ\n")

    def env_ready_callback(self, msg):
        """Update environment readiness state."""
        self.env_ready = msg.data
        if self.env_ready:
            self.get_logger().info("Environment is ready. Vehicle Interface Node can now proceed.")
            
    def publish_hsd_command(self):
        # Get the current waypoint
        waypoint = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f"Current waypoint  {self.current_waypoint_index}: {waypoint}")

        # Calculate heading and distance
        direction = waypoint[:2] - self.current_position[:2]  # x, y difference
        distance = np.linalg.norm(direction)

        # Dynamic distance threshold based on speed
        speed = 700 # min(distance * 5, 2)  # Adjust scaling factor as needed
        # distance_threshold = max(1.0, speed * 0.5)  # Dynamic threshold

        if distance > self.distance_threshold:  # Check if waypoint is reached
            heading = np.degrees(np.arctan2(direction[1], direction[0])) % 360
            depth = waypoint[2]  # Negative depth as per HoloOcean convention

            # Publish commands
            self.publish_depth(depth)
            self.publish_heading(heading)
            self.publish_speed(speed)
            self.get_logger().info(f"Published -- Depth: {depth}, Heading: {heading}, Speed: {speed}, Distance to waypoint  {self.current_waypoint_index}: {distance}")
        else:
            # Move to next waypoint if close enough
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
            logger.opt(colors=True).info(f"<green>Reached waypoint {self.current_waypoint_index}, moving to next!</green>")

            # Optional: Add a small delay or stabilization period
            time.sleep(1)  # Adjust the delay as needed
        
    # def publish_hsd_command(self):
    #     """Send heading-speed-depth (HSD) commands based on the next waypoint."""
        
    #     # Ensure the interface is initialized before drawing waypoints
    #     if not self.env_ready:
    #         self.get_logger().info("Waiting for environment readiness signal.")
    #         return  # Wait until the environment is ready

                
    #     waypoint = self.waypoints[self.current_waypoint_index]
    #     self.get_logger().info(f"this is current waypoint: {waypoint}")    

    #     # Compute distance and heading to waypoint
    #     direction = waypoint[:2] - self.current_position[:2]  # x, y difference
    #     self.get_logger().info(f"Computed direction: {direction}")
        
    #     distance = np.linalg.norm(direction)
    #     self.get_logger().info(f"Computed distance: {distance}")
        
    #     heading = self.calculate_heading(direction)
    #     self.get_logger().info(f"Computed heading: {heading}")
        
    #     distance = np.linalg.norm(direction)
    #     self.get_logger().info(f"Computed vector norm distance: {distance}")
    #     self.get_logger().info(f"distance_threshold: {self.distance_threshold}")
        

    #     if distance > self.distance_threshold:  # Check if waypoint is reached
    #         heading = self.calculate_heading(direction)
    #         depth = waypoint[2]  # Negative depth as per HoloOcean convention
    #         speed = self.calculate_speed(distance)

    #         # Publish commands
    #         self.publish_depth(depth)
    #         self.publish_heading(heading)
    #         self.publish_speed(speed)


    #         self.get_logger().info(f"Published -> Depth: {depth}, Heading: {heading}, Speed: {speed}, Distance to waypoint {self.current_waypoint_index}: {distance}")
    #         self.get_logger().info(f"Waypoint direction: {direction}, distance: {distance}, heading: {heading}")

    #     else:
    #         # Move to next waypoint if close enough
    #         self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
    #         self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}, moving to next!")

    def draw_waypoints(self):
        """Draw waypoints only after the first command is sent."""
        self.get_logger().info("First command sent, now drawing waypoints.")
        for waypoint in self.waypoints:
            self.interface.draw_point(waypoint, color=[255, 0, 0], thickness=5, lifetime=0)
        self.get_logger().info("Waypoints successfully drawn.")
            
    def log_position(self, position, waypoint):
        """Log the vehicle's position and current waypoint."""
        time = self.get_clock().now().to_msg()
        log_line = f"{time.sec}.{time.nanosec},{position[0]},{position[1]},{position[2]},{waypoint[0]},{waypoint[1]},{waypoint[2]}\n"
        self.log_file.write(log_line)
                    
    def state_callback(self, msg):
        """Update the current position from state estimation."""
        self.current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        self.log_position(self.current_position, self.waypoints[self.current_waypoint_index])
        self.get_logger().info(f"Updated position: {self.current_position}")

    def calculate_heading(self, direction):
        """Calculate the desired heading based on waypoint direction."""
        heading = np.degrees(np.arctan2(direction[1], direction[0])) % 360
        return heading if heading <= 180 else heading - 360  # Map to [-180, 180] range

    def calculate_speed(self, distance):
        """Dynamically adjust speed based on distance to waypoint."""
        return 2 # min(3.5, max(0.5, distance / 5.0))  # Scale speed for smoother transitions

    def publish_depth(self, depth):
        depth_msg = Float64()
        depth_msg.data = float(depth)
        self.depth_publisher.publish(depth_msg)

    def publish_heading(self, heading):
        heading_msg = Float64()
        heading_msg.data = float(heading)
        self.heading_publisher.publish(heading_msg)

    def publish_speed(self, speed):
        speed_msg = Float64()
        speed_msg.data = float(speed)
        self.speed_publisher.publish(speed_msg)

def main(args=None):
    rclpy.init(args=args)
    vehicle_interface_node = VehicleInterfaceNode()
    rclpy.spin(vehicle_interface_node)

    vehicle_interface_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
