import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from scipy.spatial.distance import euclidean
import numpy as np

from loguru import logger
class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Parameters
        self.declare_parameter('waypoints', [])
        self.declare_parameter('tolerance', 1.0)
        
        # Retrieve parameters
        # self.waypoints = self.get_parameter('waypoints').get_parameter_value().double_array_value
        # self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value

        # self.waypoints = [
        #             [15.0, 15.0, 1.0],   # Waypoint 1
        #             [-15.0, 15.0, 1.0],  # Waypoint 2
        #             [-15.0, -15.0, 1.0], # Waypoint 3
        #             [15.0, -15.0, 1.0]   # Waypoint 4
        # ]
        
        self.waypoints = [
                    [5.0, 1.0, 0.0],   # Waypoint 1
                    [-5.0, 1.0, 0.0],  # Waypoint 2
                    [-5.0, -1.0, 0.0], # Waypoint 3
                    [5.0, -1.0, 0.0]   # Waypoint 4
        ]
        
        self.tolerance = 1
        
        # Current position
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_yaw = 0.0  # Store current yaw

        # Subscribers
        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            'dead_reckon',
            self.pose_callback,
            10
        )

        # Publishers
        self.depth_publisher = self.create_publisher(Float64, 'depth', 10)
        self.heading_publisher = self.create_publisher(Float64, 'heading', 10)
        self.speed_publisher = self.create_publisher(Float64, 'speed', 10)

        # Control Timer
        self.control_timer = self.create_timer(1.0, self.control_loop)

        # Waypoint Tracking
        self.current_waypoint_index = 0

    def pose_callback(self, msg):
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        self.current_position[2] = msg.pose.pose.position.z
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.degrees(np.arctan2(siny_cosp, cosy_cosp))
        
        # Debugging: Log current position and yaw
        self.get_logger().info(f"Current Position: {self.current_position}, Yaw: {self.current_yaw}")

    def control_loop(self):
        if self.current_waypoint_index >= len(self.waypoints):
            logger.opt(colors=True).info(f'<green>✅ All waypoints reached. Stopping movement.</green>')
            return

        target = np.array(self.waypoints[self.current_waypoint_index])
        delta_pos = target - self.current_position
        distance = np.linalg.norm(delta_pos[:2])  # Ignore depth for now

        # ✅ Fix 1: Stronger tolerance when switching waypoints
        base_tolerance = 0.8  # Lower tolerance for small area
        dynamic_tolerance = max(base_tolerance, distance * 0.02)  # Scaled tolerance
        if distance <= dynamic_tolerance:
            logger.opt(colors=True).info(f'<green>🎯 Waypoint {self.current_waypoint_index} reached! Switching to next waypoint.</green>')
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                logger.opt(colors=True).info(f'<green>✅ All waypoints reached. Stopping movement.</green>')
                return
            return

        # ✅ Fix 2: Calculate absolute heading to next waypoint
        target_heading = np.degrees(np.arctan2(delta_pos[1], delta_pos[0]))
        heading_error = target_heading - self.current_yaw

        # Normalize heading error to [-180, 180]
        heading_error = (heading_error + 180) % 360 - 180  

        # ✅ Fix 3: Apply Stronger Heading Correction (Proportional-Derivative Control)
        Kp_heading = 1.8 if abs(heading_error) > 30 else 1.0  # Increase gain for large errors
        Kd_heading = 0.5  # Damping factor
        heading_change = Kp_heading * heading_error - Kd_heading * self.current_yaw

        heading_command = target_heading + heading_change
        heading_command = (heading_command + 180) % 360 - 180  # Normalize

        # ✅ Fix 4: Limit heading adjustments per step
        MAX_HEADING_STEP = 15.0  # Stronger turn correction
        heading_command = np.clip(heading_command, self.current_yaw - MAX_HEADING_STEP, self.current_yaw + MAX_HEADING_STEP)

        # ✅ Fix 5: Reduce Speed if Large Heading Error
        MIN_SPEED = 0.5
        MAX_SPEED = 2.0  
        if abs(heading_error) > 30:
            speed_command = MIN_SPEED  # Slow down if large turn required
        else:
            speed_command = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * (1 - abs(heading_error) / 90)

        # Set depth (invert Z for correct depth setting)
        depth = -target[2]

        # Publish speed
        speed_msg = Float64()
        speed_msg.data = speed_command
        self.speed_publisher.publish(speed_msg)

        # Publish heading
        heading_msg = Float64()
        heading_msg.data = heading_command
        self.heading_publisher.publish(heading_msg)

        # Publish depth
        depth_msg = Float64()
        depth_msg.data = depth
        self.depth_publisher.publish(depth_msg)

        self.get_logger().info(f'🚀 Moving to waypoint {self.current_waypoint_index}: '
                            f'Heading {heading_command:.2f}, Speed {speed_command:.2f}, '
                            f'Depth {depth:.2f}, Distance {distance:.2f}')



    # def control_loop(self):
    #     if self.current_waypoint_index >= len(self.waypoints):
    #         self.get_logger().info('All waypoints reached.')
    #         return

    #     target = np.array(self.waypoints[self.current_waypoint_index])
    #     delta_pos = target - self.current_position
    #     distance = np.linalg.norm(delta_pos[:2])  # Ignore depth for now

    #     WAYPOINT_REACHED_THRESHOLD = 2.0  # Stop adjusting if within 2 meters
        
    #     # if distance <= self.tolerance:
    #     #     self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached.')
    #     #     self.current_waypoint_index += 1
    #     #     return
    #     if distance <= WAYPOINT_REACHED_THRESHOLD:
    #         self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached. Switching to next waypoint.')
    #         self.current_waypoint_index += 1
    #         if self.current_waypoint_index >= len(self.waypoints):
    #             self.get_logger().info('All waypoints reached. Stopping movement.')
    #             return

    #     # Compute the heading error
    #     target_heading = np.degrees(np.arctan2(delta_pos[1], delta_pos[0]))
    #     heading_error = target_heading - self.current_yaw

    #     # Normalize to [-180, 180] degrees
    #     heading_error = (heading_error + 180) % 360 - 180  

    #     # # Apply proportional heading correction
    #     # Kp = 0.5  # Adjust this gain for better correction
    #     # heading_command = self.current_yaw + Kp * heading_error
        
    #     # Reduce heading gain when close to the waypoint
    #     Kp_heading = 0.8 if distance > 5 else 0.4  # Lower gain when close
    #     heading_command = self.current_yaw + Kp_heading * heading_error
    #     heading_command = (heading_command + 180) % 360 - 180  # Normalize to [-180, 180]


    #     # Ensure the heading stays within the valid range
    #     heading_command = (heading_command + 180) % 360 - 180  # Normalize to [-180, 180]

    #     # Set depth
    #     depth = -target[2]

    #     # Command to go forward at a set speed
    #     speed = Float64()
    #     # speed.data = speed.data = 2.0 # max(1.0, min(3.0, distance / 5)) #1.5  # m/s
    #     MIN_SPEED = 0.5
    #     MAX_SPEED = 2.0  # Your desired max speed
    #     distance_factor = min(distance / 10, 1)  # Scale factor (1 when far, 0.1 when close)
    #     speed.data = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * distance_factor
    #     self.speed_publisher.publish(speed)

    #     # Set heading
    #     heading_msg = Float64()
    #     heading_msg.data = heading_command
    #     self.heading_publisher.publish(heading_msg)

    #     # Set depth
    #     depth_msg = Float64()
    #     depth_msg.data = depth
    #     self.depth_publisher.publish(depth_msg)

    #     self.get_logger().info(f'Moving to waypoint {self.current_waypoint_index}: Heading {heading_command:.2f}, Speed : {speed.data} , Depth {depth:.2f}, Distance {distance:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()