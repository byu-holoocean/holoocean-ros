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
        self.state_pub = self.create_publisher(Odometry, 'dead_reckon', 10)

        timer_frequency = 10
        timer_period = 1/timer_frequency  # seconds publish vehicle status update
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.vel_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            'DVLSensorVelocity',
            # 'VelocitySensor',
            self.vel_callback,
            10)
        self.body_frame_velocity = True

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
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.init_yaw = None
        self.depth = 0.0  # Initialize depth
        self.last_time = None
        self.initial_state = False

    def vel_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time

        if self.body_frame_velocity:
            velocity_body = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
            #Check this order
            body_to_world = Rotation.from_euler('zyx', [self.yaw, self.pitch, self.roll], True).as_matrix()
            # body_to_world = np.transpose(matrix_world_to_body)
            #check this order 
            self.velocity = np.matmul(body_to_world, velocity_body)
        else:
            self.velocity[0] = msg.twist.twist.linear.x
            self.velocity[1] = msg.twist.twist.linear.y
            self.velocity[2] = msg.twist.twist.linear.z

        # Integrate velocities using RK45
        def derivatives(t, state, vx, vy, vz):
            return [vx, vy, vz]
        
        sol = solve_ivp(derivatives, [0, dt], self.position, args=(self.velocity[0], self.velocity[1],self.velocity[2]), method='RK45')
        self.position[0], self.position[1], self.position[2] = sol.y[:, -1]

        self.last_time = current_time

    def rotation_callback(self, msg):
        # Convert the incoming vector to roll, pitch, yaw (RPY) using 'xyz' convention
        RPY = Rotation.from_euler('xyz', [msg.vector.x, msg.vector.y, msg.vector.z], degrees=True).as_euler('zyx', degrees=True)

        # Initialize yaw if not already set
        if self.init_yaw is None:
            self.init_yaw = RPY[0]
            return
        
        # Extract roll, pitch, and calculate the adjusted yaw
        self.roll = RPY[2]
        self.pitch = RPY[1]
        
        # Adjust yaw based on initial yaw
        self.yaw = RPY[0] - self.init_yaw
        
        # Wrap yaw to the range [-π, π] (or [-180°, 180°] in degrees)
        self.yaw = (self.yaw + 180) % 360 - 180
        self.initial_state = True

    def depth_callback(self, msg):
        self.depth = msg.pose.pose.position.z

    def timer_callback(self):
        if self.initial_state:
            quaternion = Rotation.from_euler('zyx', [self.yaw, self.pitch, self.roll], True).as_quat()

            # Publish new state
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'

            msg.pose.pose.position.x = float(self.position[0])
            msg.pose.pose.position.y = float(self.position[1])
            msg.pose.pose.position.z = float(self.position[2])
            msg.pose.pose.orientation.x = float(quaternion[0])
            msg.pose.pose.orientation.y = float(quaternion[1])
            msg.pose.pose.orientation.z = float(quaternion[2])
            msg.pose.pose.orientation.w = float(quaternion[3])

            msg.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,  # High uncertainty in Z
                            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,  # High uncertainty in roll
                            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,  # High uncertainty in pitch
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.01] # Confidence in yaw


            # msg.twist.twist.linear.x = float(self.velocity[0])
            # msg.twist.twist.linear.y = float(self.velocity[1])
            # msg.twist.twist.linear.z = float(self.velocity[2])

            self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RKStateEstimate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
