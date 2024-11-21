from holoocean_main.holoocean_interface import HolooceanInterface
from holoocean.vehicle_dynamics import *
from holoocean.dynamics import *

import rclpy
from rclpy.node import Node

from holoocean_main.sensor_data_encode import UCommand
from std_msgs.msg import Header
from std_srvs.srv import Empty
from frost_interfaces.msg import DesiredHeading, DesiredDepth


class MoosFinsNode(Node):
    
    def __init__(self):
        super().__init__('moos_fins_node')
        
        ######## START HOLOOCEAN INTERFACE ###########
        self.declare_parameter('params_file', '')
        file_path = self.get_parameter('params_file').get_parameter_value().string_value

        self.interface = HolooceanInterface(file_path, node=self)

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
        # Positive value for increasing depth (meters)
        self.depth_sub = self.create_subscription(
            DesiredDepth,
            'desired_depth',
            self.depth_callback,
            10
        )
        self.depth_goal = 0
        
        # Heading in degrees (-180, 180) centered at NORTH?? 
        self.heading_sub = self.create_subscription(
            DesiredHeading,
            'desired_heading',
            self.heading_callback,
            10
        )
        self.heading_goal = 0

        #TODO: Create a service to reset the enviornment
        # reset the control surfaces to 0 as well
        self.srv = self.create_service(Empty, 'reset_holoocean', self.reset)

        ######### CUSTOM SIMULATION INIT ########

        self.vehicle = threeFinInd(self.interface.scenario, 'auv0','manualControl')
        self.torpedo_dynamics = FossenDynamics(self.vehicle,self.interface.get_time_warp_period())  

   
    def tick_callback(self):
        #Tick the envionment and publish data as many times as requested
        state = self.interface.tick(self.accel)
    
        self.accel = self.torpedo_dynamics.update(state) #Calculate accelerations to be applied to HoloOcean agent

        self.interface.publish_sensor_data(state)


        self.draw_arrow(state)

        #TODO: Get the desired heading arrow 

        #PASS IN THE VALUES FOR THE ARROWS in yaml file
        self.interface.env.draw_arrow([40,-40,0], end=[40, -40, 5], color=[0,0,255], thickness=50, lifetime=self.interface.get_period()+0.01)
        self.interface.env.draw_arrow([-40,-20,0], end=[-40, -20, 5], color=[255,0,0], thickness=50, lifetime=self.interface.get_period()+0.01)
        self.interface.env.draw_arrow([-150,0,0], end=[-150, 0, 5], color=[0,255,0], thickness=50, lifetime=self.interface.get_period()+0.01)

    def callback_set_fins(self, msg):
        u_control = np.zeros(self.vehicle.dimU, np.float64)
        for i in range(self.vehicle.dimU - 1):
            u_control[i] = msg.fin[i]
            # print(i, u_control[i])
        
        u_control = np.deg2rad(u_control)
        
        u_control[-1] = float(msg.thruster)

        self.torpedo_dynamics.set_u_control_rad(u_control)     

    def reset(self, request, response):
        u_control = np.zeros(self.vehicle.dimU, np.float64)
        self.torpedo_dynamics.set_u_control_rad(u_control)
        self.get_logger().info('Service called and callback executed.')
        self.interface.env.reset()

        # Do any actions needed on service call
        return response

    def create_publishers(self):
        for sensor in self.interface.sensors:
            sensor.publisher = self.create_publisher(sensor.message_type, sensor.name, 10)

    def depth_callback(self,msg):
        self.depth_goal = - msg.desired_depth

    def heading_callback(self,msg):

        #TODO: Change coordinate frame for the heading goal to reflect the NWU in holoocean
        self.heading_goal =  msg.desired_heading

    def draw_arrow(self, state):
        # For plotting HSD and arrows 
        depth = self.depth_goal
        heading = self.heading_goal

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
            

def main(args=None):
    rclpy.init(args=args)
    node = MoosFinsNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
