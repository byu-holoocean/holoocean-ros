import rclpy
from rclpy.node import Node
from holoocean_main.interface.holoocean_interface import *
from ament_index_python.packages import get_package_share_directory
import os

from holoocean_interfaces.msg import ControlCommand, DesiredCommand, AgentCommand
from holoocean_interfaces.srv import SetControlMode
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Time


package_name = 'holoocean_main'

CONTROL_MODE_MAP = {
    0: "manualControl",
    1: "depthHeadingAutopilot",
    2: "headingAutopilot",
    3: "depthAutopilot"
}

class HoloOceanNode(Node):
    
    def __init__(self):
        super().__init__('holoocean_node')
        
        self.declare_parameter('publish_commands', True)
        publish_commands = self.get_parameter('publish_commands').get_parameter_value().bool_value
        self.declare_parameter('show_viewport', True)
        show_viewport = self.get_parameter('show_viewport').get_parameter_value().bool_value

        self.declare_parameter('relative_path', True)
        relative_path = self.get_parameter('relative_path').get_parameter_value().bool_value
        self.declare_parameter('scenario_path', '')
        scenario_path = self.get_parameter('scenario_path').get_parameter_value().string_value
        if relative_path:
            package_dir = Path(get_package_share_directory('holoocean_main'))
            config_file = os.path.join(package_dir, scenario_path)
            print("config: ", config_file)
        else:
            config_file = scenario_path

        ######## START HOLOOCEAN INTERFACE ###########
        self.interface = HolooceanInterface(config_file, node=self, publish_commands=publish_commands, show_viewport=show_viewport)
        # TODO: Look into threading and callbacks for the HoloOcean simulator
        # TODO: Probably just want to run it as fast as I can
        # TODO: Set the time warp in the ros params
        # TODO: See if that affects the visuals if it is not ticking and returning quickly
        self.timer = self.create_timer(self.interface.get_time_warp_period(), self.tick_callback)
        self.get_logger().info('Tick Started')

        self.accel = np.array(np.zeros(6),float)
        
        ######## CUSTOM SUBSCRIBERS ############
        
        self.ucommand_sub = self.create_subscription(ControlCommand, 'command/control', self.u_control_callback, 10)
        self.ucommand_sub = self.create_subscription(AgentCommand, 'command/agent', self.agent_command_callback, 10)

        self.depth_sub = self.create_subscription(DesiredCommand, 'depth', self.interface.depth_callback, 10)
        self.heading_sub = self.create_subscription(DesiredCommand, 'heading', self.interface.heading_callback, 10)
        self.speed_sub = self.create_subscription(DesiredCommand, 'speed', self.interface.speed_callback, 10)

        self.clock_pub = self.create_publisher(Time, '/clock', 10)

        # Services
        self.reset_srv = self.create_service(Trigger, 'reset', self.reset)
        self.control_mode_srv = self.create_service(SetControlMode, 'control_mode', self.control_mode_callback)


    def tick_callback(self):
        #Tick the envionment and publish data as many times as requested
        time = self.interface.tick()


        # Publish the sim time
        time_msg = Time()
        time_msg.sec = int(time)
        time_msg.nanosec = int((time - int(time)) * 1e9)

        self.clock_pub.publish(time_msg)
        


    # TODO create TF Transform publisher
    def reset(self, request, response):
        # TODO reset just one agent
        self.interface.reset_enviornment()

        response.message = 'Resetting the HoloOcean Enviornment'
        response.success = True

        return response
    
    def control_mode_callback(self, request, response):
        # Get the associated 
        mode_string = CONTROL_MODE_MAP.get(request.mode, None)

        if mode_string is not None:
            self.interface.set_control_mode(request.vehicle_name, mode_string)

            response.message = f'Setting Control Mode to {mode_string}'
            response.success = True
        else:
            response.message = f'Unkown Control Mode value: {request.mode}' 
            response.success = False

        return response
    
    def u_control_callback(self, msg):
        vehicle_name = msg.header.frame_id
        u_control_list = list(msg.cs)  # Convert from array.array to list
        self.interface.set_u_control(vehicle_name, u_control_list)

    def agent_command_callback(self, msg):
        vehicle_name = msg.header.frame_id
        agent_command = np.array(msg.command)
        self.interface.set_agent_command(vehicle_name, agent_command)

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
    node = HoloOceanNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()