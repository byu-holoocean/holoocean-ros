from holoocean_main.holoocean_interface import HolooceanInterface
from holoocean.vehicle_dynamics import *
from holoocean.dynamics import *

import rclpy
from rclpy.node import Node

from holoocean_interfaces.msg import UCommand
from std_msgs.msg import Header


class FinsNode(Node):
    
    def __init__(self):
        super().__init__('fins_node')
        
        ######## START HOLOOCEAN INTERFACE ###########
        self.declare_parameter('params_file', '')
        file_path = self.get_parameter('params_file').get_parameter_value().string_value

        self.interface = HolooceanInterface(file_path)

        self.create_publishers() #Holoocean Publishers
        self.timer = self.create_timer(self.interface.get_time_warp_period(), self.tick_callback)
        self.get_logger().info('Tick Started')

        self.accel = np.array(np.zeros(6),float)
        
        ######## CUSTOM SUBSCRIBERS ############
        
        #Control Surfaces Sub:
        self.subscription = self.create_subscription(
            UCommand,
            'ControlCommand',
            self.callback_set_fins,
            10
        )

        ######### CUSTOM SIMULATION INIT ########

        self.vehicle = threeFinInd(self.interface.scenario, 'auv0','manualControl')
        self.torpedo_dynamics = FossenDynamics(self.vehicle,self.interface.get_time_warp_period())  

   
    def tick_callback(self):
        #Tick the envionment and publish data as many times as requested
        state = self.interface.tick(self.accel)
    
        self.accel = self.torpedo_dynamics.update(state) #Calculate accelerations to be applied to HoloOcean agent

        self.interface.publish_sensor_data(state)

    def callback_set_fins(self, msg):
        u_control = np.zeros(self.vehicle.dimU, np.float64)
        for i in range(self.vehicle.dimU - 1):
            u_control[i] = msg.fin[i]
            # print(i, u_control[i])
        
        u_control = np.deg2rad(u_control)
        
        u_control[-1] = float(msg.thruster)

        self.torpedo_dynamics.set_u_control_rad(u_control)     

    def create_publishers(self):
        for sensor in self.interface.sensors:
            sensor.publisher = self.create_publisher(sensor.message_type, sensor.name, 10)
            

def main(args=None):
    rclpy.init(args=args)
    node = FinsNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
