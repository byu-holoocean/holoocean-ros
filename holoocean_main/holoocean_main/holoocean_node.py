#Import the create messgaes file
# from holoocean_main.sensor_data_converter import convert_to_msg, sensor_keys, 
from holoocean_main.sensor_data_encode import encoders, multi_publisher_sensors
from holoocean.dynamics_controls.interface import VehicleInterface

import rclpy
from rclpy.node import Node

from holoocean_interfaces.msg import HSD
from std_msgs.msg import Header


class HolooceanNode(Node):
    
    def __init__(self):
        super().__init__('holoocean_node')
        
        self.declare_parameter('params_file', '')
        
        file_path = self.get_parameter('params_file').get_parameter_value().string_value

        self.interface = VehicleInterface(file_path)
        
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
        #TODO: Paramter from yaml
        period = self.interface.get_warp_period()
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
        scenario = self.interface.get_scenario()  # Get full scenario from environment

        if len(scenario["agents"]) > 1:
            self.multi_agent_scenario = True
            print("Give sensors unique names that are reported on multiple agents")
        else:
            self.multi_agent_scenario = False

        self.sensors = []

        for agent in scenario["agents"]:
            for sensor in agent["sensors"]:
                if sensor['ros_publish']:
                    sensor_type = sensor['sensor_type']

                    if sensor_type in multi_publisher_sensors:
                        for suffix in multi_publisher_sensors[sensor_type]:
                            full_type = f"{sensor['sensor_type']}{suffix}"
                            full_name = f"{sensor['sensor_name']}{suffix}"
                            
                            encoder_class = encoders.get(full_type)
                            sensor_copy = sensor.copy()  # Create a copy of the sensor dictionary
                            sensor_copy['sensor_name'] = full_name
                            sensor_copy['agent_name'] = agent['agent_name']

                            self.sensors.append(encoder_class(sensor_copy))
                    else:
                        sensor['agent_name'] = agent['agent_name']
                        encoder = encoders[sensor['sensor_type']]
                        self.sensors.append(encoder(sensor))

        self.create_publishers()

    def create_publishers(self):
        for sensor in self.sensors:
            sensor.publisher = self.create_publisher(sensor.message_type, sensor.name, 10)
                   
        
    def publish_sensor_data(self, state):
        
        for sensor in self.sensors:

            try:
                if self.multi_agent_scenario:
                    msg = sensor.encode(state[sensor.agent_name][sensor.name])

                else:
                    msg = sensor.encode(state[sensor.type])

                # Header
                msg.header.stamp.sec = int(state['t'])  # Set seconds part from state['t']
                msg.header.stamp.nanosec = int((state['t'] - msg.header.stamp.sec) * 1e9)

                sensor.publisher.publish(msg)
            except KeyError:
                # Handle the case where sensor data is not available
                # self.get_logger().info(f"No data for sensor: {sensor['sensor_name']}")
                pass
            except Exception as e:
                # Handle other exceptions
                self.get_logger().error(f"Error processing sensor: {sensor.name}, type: {sensor.type}, error: {str(e)}")
            

    def adjust_timer(self, new_period):
        self.get_logger().info(f'Adjusting timer period to {new_period} seconds')
        self.timer.cancel()
        self.timer = self.create_timer(new_period, self.tick_callback)
    

    def tick_callback(self):
        #Tick the envionment and publish data as many times as requested
        if self.callback_in_progress:
            self.get_logger().warn('Callback is being called faster than it finishes!')

        self.callback_in_progress = True 
      
        state = self.interface.tick()
        self.publish_sensor_data(state)
    
        self.callback_in_progress = False 
        

def main(args=None):
    rclpy.init(args=args)
    node = HolooceanNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
