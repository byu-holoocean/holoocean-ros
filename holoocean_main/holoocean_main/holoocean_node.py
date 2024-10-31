from holoocean_main.holoocean_interface import HolooceanInterface, np

import rclpy
from rclpy.node import Node

from holoocean_interfaces.msg import HSD
from std_msgs.msg import Header


class HolooceanNode(Node):
    
    def __init__(self):
        super().__init__('holoocean_node')
        
        self.declare_parameter('params_file', '')
        
        file_path = self.get_parameter('params_file').get_parameter_value().string_value

        self.interface = HolooceanInterface(file_path, node=self)
        
        #Create list of sensor publishers that have ros_publish=True based on scenario:
        self.sensors = []
        self.sensor_publisher_create()
        
        #Depth heading subscriber:
        self.subscription = self.create_subscription(
            HSD,
            'desiredHSD',
            self.callback_set_controller,
            10
        )

        #TODO: Make sure it doesnt tick to fast
        #Tick Timer
        period = self.interface.get_time_warp_period()
        print("Time Warp Period:", period)
        self.timer = self.create_timer(period, self.tick_callback)
        self.callback_in_progress = False
        self.get_logger().info('Tick Started')

    def callback_set_controller(self, msg):
        # self.get_logger().info('Controller Received: {}'.format(msg))
        #TODO: Change speed to a m/s controller maybe use DVL?
        #TODO: Controller use sensor data instead of simulation data
        self.interface.send_command(msg.depth,msg.heading,msg.speed)
        
    def sensor_publisher_create(self):
        self.sensors = self.interface.create_sensor_list()

        self.create_publishers()

    def create_publishers(self):
        for sensor in self.sensors:
            sensor.publisher = self.create_publisher(sensor.message_type, sensor.name, 10)
                   
    def publish_sensor_data(self, state):
        self.interface.publish_sensor_data(state)
            
    def adjust_timer(self, new_period):
        self.get_logger().info(f'Adjusting timer period to {new_period} seconds')
        self.timer.cancel()
        self.timer = self.create_timer(new_period, self.tick_callback)
    

    def tick_callback(self):
        #Tick the envionment and publish data as many times as requested
        
        #PASS IN A COMMAND TO THE TICK
        command = np.array(np.zeros(6),float)

        state = self.interface.tick(command)
        self.publish_sensor_data(state)
    
        

def main(args=None):
    rclpy.init(args=args)
    node = HolooceanNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
