from holoocean.vehicle_dynamics.torpedo import *
from holoocean_main.holoocean_interface import HolooceanInterface
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, Vector3Stamped
from holoocean_interfaces.msg import UCommand


class ControllerExample(Node):

    def __init__(self):
        super().__init__('controller_node')

        # Vehicle Setup
        self.declare_parameter('params_file', '')
        file_path = self.get_parameter('params_file').get_parameter_value().string_value

        interface = HolooceanInterface(file_path, init=False)
        
        # self.declare_parameter('params_file', '') #TORPEDO TYPE
        # torpedo name get
        self.torpedo = threeFinInd(interface.get_scenario(),vehicle_name='auv0')

        ############ HSD subscribers: ############
        self.depth = 0.0
        self.heading = 0.0
        self.speed = 0.0

        # Positive value for increasing depth (meters)
        self.depth_sub = self.create_subscription(
            Float64,
            'depth',
            self.depth_callback,
            10
        )
        
        # Heading in degrees (-180, 180) centered at NORTH?? 
        self.heading_sub = self.create_subscription(
            Float64,
            'heading',
            self.heading_callback,
            10
        )

        # Speed in x body frame (forward, m/s)
        self.speed_sub = self.create_subscription(
            Float64,
            'speed',
            self.speed_callback,
            10
        )

        ############ Data Subscriber ############
        self.nu = np.zeros(3, np.float64)
        self.roll = 0.0
        self.pitch = 0.0 
        self.yaw = 0.0
        self.z = 0.0

        #DVL Sensor returns velocity in body frame
        self.vel_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            'DVLSensorVelocity',
            self.vel_callback,
            10)

        #Roation Sensor return rotation 
        self.rotation_sub = self.create_subscription(
            Vector3Stamped,
            'RotationSensor',
            self.rotation_callback,
            10)

        self.depth_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'DepthSensor',
            self.depth_sensor_callback,
            10)
        
        ######## Control Surfaces Publishers #########

        self.publisher = self.create_publisher(UCommand, 'ControlCommand', 10)


        ######## Control loop timer ###############
       
        self.timer_period =   interface.get_time_warp_period()
        self.timer_control = self.create_timer(self.timer_period, self.control_loop)

    ###### Control Loop #########
    #TODO: Check the timing and make sure the control loop runs at correct hz
    #Consider making a flag to indicate that the data is correct
    def control_loop(self):
        #not used for this controller
        x = y = 0
        z = self.z
        eta = [x, y, z, self.roll, self.pitch, self.yaw ]

        #DO SOMETHING WITHOUT THE IMU
        #Calculate the TIME WITH THE TIME FUNCTION

        u_control = self.torpedo.depthHeadingAutopilot(eta, self.nu, self.timer_period, imu=False)

        msg = UCommand()
        msg.fin = [0.0,0.0,0.0,0.0]
        for i in range(self.torpedo.dimU - 1):
            msg.fin[i] = np.rad2deg(u_control[i])
        
        #CHECK: Make sure it can access nMax
        # mapped_thrust = self.map_to_100(u_control[-1], self.torpedo.nMax)
        # msg.thruster = int(mapped_thrust)
        msg.thruster = int(u_control[-1])

        self.publisher.publish(msg)

    def map_to_100(self, value, max_value):
        if max_value == 0:
            raise ValueError("max_value must be greater than 0")
        return (value / max_value) * 100       

    ### Data Sensor Callback ####
    def vel_callback(self, msg):
        self.nu[0] = msg.twist.twist.linear.x
        self.nu[1] = msg.twist.twist.linear.y
        self.nu[2] = msg.twist.twist.linear.z

    def rotation_callback(self, msg):
        #TODO: Make sure the euler angles are switched to zyx in radians
        data = [msg.vector.x, msg.vector.y, msg.vector.z]
        corrected = Rotation.from_euler('xyz',data, True).as_euler('zyx', False)
        
        self.roll = corrected[0]
        self.pitch = corrected[1]
        self.yaw = corrected[2]

    def depth_sensor_callback(self, msg):
        self.z = msg.pose.pose.position.z

    ### HSD Callback Functions ###
    def depth_callback(self, msg):
        #TODO Check to make sure change to coordinate frame of the depth goal to positive downward
        
        self.torpedo.set_depth_goal(msg.data)

    def heading_callback(self, msg):
        #TODO: Controller wants heading from -180 to 180 in degrees centered at north
        self.torpedo.set_heading_goal(msg.data)

    def speed_callback(self, msg):
        self.torpedo.set_surge_goal(msg.data)


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