import threading
import rclpy
from rclpy.node import Node
from holoocean_main.interface.holoocean_interface import *
from ament_index_python.packages import get_package_share_directory
import os

from holoocean_interfaces.msg import ControlCommand, DesiredCommand, AgentCommand
from holoocean_interfaces.srv import SetControlMode
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from rosgraph_msgs.msg import Clock


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
        # NOTE: Maybe move this to the ros message to draw the arrow or not
        self.declare_parameter('draw_arrow', True)
        draw_arrow = self.get_parameter('draw_arrow').get_parameter_value().bool_value

        self.declare_parameter('draw_waypoints', False)

        draw_waypoints = self.get_parameter('draw_waypoints').get_parameter_value().bool_value


        self.declare_parameter('render_quality', -1)
        render_quality = self.get_parameter('render_quality').get_parameter_value().integer_value
        # Set Render quality to None because of bug with frames per sec set to false when render quality is set
        if render_quality == -1:
            render_quality = None 

        self.declare_parameter('relative_path', True)
        relative_path = self.get_parameter('relative_path').get_parameter_value().bool_value
        self.declare_parameter('scenario_path', '')
        scenario_path = self.get_parameter('scenario_path').get_parameter_value().string_value
        if scenario_path == '':
            raise ValueError("The 'scenario_path' parameter cannot be blank.")
        if relative_path:
            package_dir = Path(get_package_share_directory('holoocean_main'))
            config_file = os.path.join(package_dir, scenario_path)
            print("Using Scenario config file: ", config_file)
        else:
            config_file = scenario_path

        ######## START HOLOOCEAN INTERFACE ###########
        #TODO dont pass the node to the interface instead have the publishers created in the node with a function
        self.interface = HolooceanInterface(config_file, node=self, publish_commands=publish_commands, show_viewport=show_viewport, arrow_flag=draw_arrow, render_quality=render_quality)

        self.accel = np.array(np.zeros(6),float)
        
        ######## CUSTOM SUBSCRIBERS ############
        
        self.ucommand_sub = self.create_subscription(ControlCommand, 'command/control', self.u_control_callback, 10)
        self.ucommand_sub = self.create_subscription(AgentCommand, 'command/agent', self.agent_command_callback, 10)

        # TODO seperate ROS and Holoocean stuff by making functions in the node that call the interface
        self.depth_sub = self.create_subscription(DesiredCommand, 'depth', self.depth_callback, 10)
        self.heading_sub = self.create_subscription(DesiredCommand, 'heading', self.heading_callback, 10)
        self.speed_sub = self.create_subscription(DesiredCommand, 'speed', self.speed_callback, 10)

        self.clock_pub = self.create_publisher(Clock, '/clock', 10)

        self.debug_points_sub = self.create_subscription(Marker, 'debug/points', self.debug_points_callback, 10)

        # Services
        self.reset_srv = self.create_service(Trigger, 'reset', self.reset)
        self.control_mode_srv = self.create_service(SetControlMode, 'control_mode', self.control_mode_callback)


        # Start the simulation ticking in a background thread
        self._sim_running = True
        self.tick_thread = threading.Thread(target=self.sim_loop, daemon=True)
        self.tick_thread.start()

        self.get_logger().info('HoloOcean simulation thread started.')



    def sim_loop(self):
        """Run the simulation step in a thread, using Unreal's internal timing."""
        while self._sim_running:
            sim_time = self.interface.tick()  # blocks until the sim step is done

            # Publish simulation time
            time_msg = Clock()
            time_msg.clock.sec = int(sim_time)
            time_msg.clock.nanosec = int((sim_time - int(sim_time)) * 1e9)
            self.clock_pub.publish(time_msg)


    def destroy_node(self):
        """Override to cleanly shut down"""
        self._sim_running = False
        self.tick_thread.join()
        super().destroy_node()

    def tick_callback(self):
        #Tick the envionment and publish data
        time = self.interface.tick()

        # Publish the sim time
        time_msg = Clock()
        time_msg.clock.sec = int(time)
        time_msg.clock.nanosec = int((time - int(time)) * 1e9)

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

    
    def depth_callback(self, msg):
        vehicle_name = msg.header.frame_id 
        self.interface.set_depth(vehicle_name, msg.data)
    
    def heading_callback(self, msg):
        vehicle_name = msg.header.frame_id 
        self.interface.set_heading(vehicle_name, msg.data)

    def speed_callback(self, msg):
        vehicle_name = msg.header.frame_id 
        self.interface.set_speed(vehicle_name, msg.data)

    def debug_points_callback(self, msg):
        # Marker.points is a list of geometry_msgs/Point
        points = [[p.x, p.y, p.z] for p in msg.points]
        
        # ROS ColorRGBA (0.0-1.0) -> HoloOcean RGB (0-255)
        colors= []
        color = [
            int(msg.color.r * 255),
            int(msg.color.g * 255),
            int(msg.color.b * 255)
        ]
        colors = [color] * len(points)
        if len(msg.colors) > 0:
            colors = []
            for c in msg.colors:
                colors.append([
                    int(c.r * 255),
                    int(c.g * 255),
                    int(c.b * 255)
                ])
            
        thickness = msg.scale.x * 100 # Holoocean (Unreal) draw_points units is in centimeters, RVIZ is in meters
        
        # ROS duration -> float seconds
        lifetime = msg.lifetime.sec + (msg.lifetime.nanosec / 1e9)
        
        # If points list is empty, use the single pose position (common for CUBE/SPHERE)
        if not points:
            points = [[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]]
        self.interface.draw_debug_points(points, colors, thickness, lifetime)
    
def main(args=None):
    rclpy.init(args=args)
    node = HoloOceanNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()