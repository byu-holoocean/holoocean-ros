from holoocean_main.holoocean_interface import HolooceanInterface
from holoocean.vehicle_dynamics import *
from holoocean.dynamics import *

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, Float64


class TorpedoNode(Node):
    
    def __init__(self):
        super().__init__('torpedo_node')
        
        ######## START HOLOOCEAN INTERFACE ###########
        self.declare_parameter('params_file', '')
        file_path = self.get_parameter('params_file').get_parameter_value().string_value

        self.interface = HolooceanInterface(file_path, node=self)

        self.create_publishers() #Holoocean Publishers
        self.timer = self.create_timer(self.interface.get_time_warp_period(), self.tick_callback)
        self.get_logger().info('Tick Started')

        self.accel = np.array(np.zeros(6),float)
        
        ######## CUSTOM SUBSCRIBERS ############
        
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

        # Speed (surge) in x body frame (forward, m/s)
        self.speed_sub = self.create_subscription(
            Float64,
            'speed',
            self.speed_callback,
            10
        )

        ######### CUSTOM SIMULATION INIT ########
        self.draw = False
        if "draw_arrow" in self.interface.scenario:
            self.draw = self.interface.scenario["draw_arrow"]
            
        self.use_rpm = False

        #Create vehicle object attached to holoocean agent with dynamic parameters 
        #TODO: Change the vehicle that is being setup from the parameters
        self.vehicle = threeFinInd(self.interface.scenario, 'auv0','depthHeadingAutopilot')
        self.torpedo_dynamics = FossenDynamics(self.vehicle, self.interface.get_period())  

   
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

    def depth_callback(self,msg):
        self.vehicle.set_depth_goal(msg.data)

    def heading_callback(self,msg):
        self.vehicle.set_heading_goal(msg.data)

    def speed_callback(self, msg):
        if self.use_rpm:
            self.vehicle.set_rpm_goal(int(msg.data))
        else:
            self.vehicle.set_surge_goal(msg.data)

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


def main(args=None):
    rclpy.init(args=args)
    node = TorpedoNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
