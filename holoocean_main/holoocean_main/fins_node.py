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
        self.set_timing() #Connect holocean timing to ros timing

        self.accel = np.array(np.zeros(6),float)
        
        ######## CUSTOM SUBSCRIBERS ############
        
        #Control Surfaces Sub:
        self.subscription = self.create_subscription(
            UCommand,
            'u_command',
            self.callback_set_fins,
            10
        )

        ######### CUSTOM SIMULATION INIT ########
        # self.draw = False
        # if "draw_arrow" in self.interface.scenario:
        #     self.draw = self.interface.scenario["draw_arrow"]

        self.vehicle = threeFinInd(self.interface.scenario, 'auv0','manualControl')
        self.torpedo_dynamics = FossenDynamics(self.vehicle,self.interface.get_tick_rate())  

   
    def tick_callback(self):
        #Tick the envionment and publish data as many times as requested
        state = self.interface.tick(self.accel)
    
        self.accel = self.torpedo_dynamics.update(state) #Calculate accelerations to be applied to HoloOcean agent

        #TODO: Dont publish commands for this node since they are already published
        # #TODO: Handle the multi agent case for the control commands here
        # fins = np.rad2deg(self.torpedo_dynamics.u_actual[:-1])
        # thruster = self.torpedo_dynamics.u_actual[-1]

        # state["ControlCommand"] = np.append(fins,thruster)

        # if self.draw:
        #     self.draw_arrow(state)

        self.interface.publish_sensor_data(state)

    def callback_set_fins(self, msg):
        u_control = np.zeros(self.dimU, np.float64)
        for i in range(self.vehicle.dimU - 1):
            u_control[i] = msg.fin[i]
        
        u_control = np.deg2rad(u_control)
        
        u_control[-1] = float(msg.thruster)
        self.torpedo_dynamics.set_u_control_rad(u_control)     

    def create_publishers(self):
        for sensor in self.interface.sensors:
            sensor.publisher = self.create_publisher(sensor.message_type, sensor.name, 10)
            
    def set_timing(self):
        #TODO: Clean this up
        self.time_warp = 1.0
        #If set to max speed time warp will be 0 because frames per sec will be 0.
        self.time_warp = self.interface.get_frame_rate() / self.interface.get_tick_rate()
        self.warp_period = self.interface.get_period() / self.time_warp
        print("Time Warp:", self.time_warp)

        period = self.warp_period
        print("Time Warp Period:", period)
        self.timer = self.create_timer(period, self.tick_callback)
        
        self.get_logger().info('Tick Started')


def main(args=None):
    rclpy.init(args=args)
    node = FinsNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
