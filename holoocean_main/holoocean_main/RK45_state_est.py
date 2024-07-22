import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation

class RKStateEstimate(Node):

    def __init__(self):
        super().__init__('RK45_state_est')
        self.state_pub = self.create_publisher(Odometry, 'vehicle_status', 10)

        timer_period = 0.01  # seconds publish vehicle status update
        self.timer = self.create_timer(timer_period, self.timer_callback)

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

        # Initialize state variables
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.depth = 0.0  # Initialize depth
        self.last_time = None

    def vel_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time

        velocity_body = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        #Check this order
        body_to_world = Rotation.from_euler('xyz', [self.roll, self.pitch, self.yaw], True).as_matrix()
        # body_to_world = np.transpose(matrix_world_to_body)
        #check this order 
        self.velocity = np.matmul(body_to_world, velocity_body)

        # Integrate velocities using RK45
        def derivatives(t, state, vx, vy):
            return [vx, vy]
        
        sol = solve_ivp(derivatives, [0, dt], self.position[:2], args=(self.velocity[0], self.velocity[1]), method='RK45')
        self.position[0], self.position[1] = sol.y[:, -1]

        self.last_time = current_time

    def rotation_callback(self, msg):
        self.roll = msg.vector.x
        self.pitch = msg.vector.y
        self.yaw = msg.vector.z

    def depth_callback(self, msg):
        self.depth = msg.pose.pose.position.z

    def timer_callback(self):
        quaternion = Rotation.from_euler('xyz', [self.roll, self.pitch, self.yaw], True).as_quat()

        # Publish new state
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = float(self.position[0])
        msg.pose.pose.position.y = float(self.position[1])
        msg.pose.pose.position.z = float(self.depth)
        msg.pose.pose.orientation.x = float(quaternion[0])
        msg.pose.pose.orientation.y = float(quaternion[1])
        msg.pose.pose.orientation.z = float(quaternion[2])
        msg.pose.pose.orientation.w = float(quaternion[3])

        msg.twist.twist.linear.x = float(self.velocity[0])
        msg.twist.twist.linear.y = float(self.velocity[1])
        msg.twist.twist.linear.z = float(self.velocity[2])

        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RKStateEstimate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
