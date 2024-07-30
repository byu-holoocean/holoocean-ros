from holoocean_main.holoocean_interface import HolooceanInterface
from holoocean.vehicle_dynamics import *
from holoocean.dynamics import *

import rclpy
from rclpy.node import Node

from holoocean_interfaces.msg import HSD
from std_msgs.msg import Header


class TorpedoNode(Node):
    
    def __init__(self):
        super().__init__('torpedo_node')
        
        ######## START HOLOOCEAN INTERFACE ###########
        self.declare_parameter('params_file', '')
        
        file_path = self.get_parameter('params_file').get_parameter_value().string_value

        self.interface = HolooceanInterface(file_path)

        self.create_publishers() #Holoocean Publishers
        self.set_timing() #Connect holocean timing to ros timing

        self.accel = np.array(np.zeros(6),float)
        
        ######## CUSTOM SUBSCRIBERS ############
        
        #Depth heading subscriber:
        self.subscription = self.create_subscription(
            HSD,
            'desiredHSD',
            self.callback_set_controller,
            10
        )

        ######### CUSTOM SIMULATION INIT ########
        self.draw = False
        if "draw_arrow" in self.interface.scenario:
            self.draw = self.interface.scenario["draw_arrow"]

        #Create vehicle object attached to holoocean agent with dynamic parameters 
        #TODO: Change the vehicle that is being setup from the parameters
        self.vehicle = threeFinInd(self.interface.scenario, 'auv0','depthHeadingAutopilot')
        self.torpedo_dynamics = FossenDynamics(self.vehicle,self.interface.get_tick_rate())  

   
    def tick_callback(self):
        #Tick the envionment and publish data as many times as requested
        state = self.interface.tick(self.accel)
    
        self.accel = self.torpedo_dynamics.update(state) #Calculate accelerations to be applied to HoloOcean agent

        #TODO: Handle the multi agent case for the control commands here
        fins = np.rad2deg(self.torpedo_dynamics.u_actual[:-1])
        thruster = self.torpedo_dynamics.u_actual[-1]

        state["ControlCommand"] = np.append(fins,thruster)

        if self.draw:
            self.draw_arrow(state)

        self.interface.publish_sensor_data(state)

    def callback_set_controller(self, msg):
        # self.get_logger().info('Controller Received: {}'.format(msg))
        self.vehicle.set_goal(msg.depth, msg.heading, msg.speed)     #Changes depth (positive depth), heading, thruster RPM goals for controller

    def create_publishers(self):
        for sensor in self.interface.sensors:
            sensor.publisher = self.create_publisher(sensor.message_type, sensor.name, 10)
            
    def draw_arrow(self, state):
        # For plotting HSD and arrows 
        depth = self.vehicle.ref_z
        heading = self.vehicle.ref_psi

        pos = state['DynamicsSensor'][6:9]  # [x, y, z]
        x_end = pos[0] + 3 * np.cos(np.deg2rad(heading))
        y_end = pos[1] - 3 * np.sin(np.deg2rad(heading))

        #change color if within 2 meters
        if abs(depth + pos[2]) <= 2.0:
            color = [0,255,0]
        else:
            color = [255,0,0]

        self.interface.env.draw_arrow(pos.tolist(), end=[x_end, y_end, pos[2]], color=[0,0,255], thickness=5, lifetime=self.interface.get_period()+0.01)
        self.interface.env.draw_arrow(pos.tolist(), end=[pos[0], pos[1], -depth], color=color, thickness=5, lifetime=self.interface.get_period()+0.01)

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
    node = TorpedoNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
