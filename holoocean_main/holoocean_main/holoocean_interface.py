import numpy as np
import holoocean

from pathlib import Path
import json
from holoocean_main.sensor_data_encode import encoders, multi_publisher_sensors

import time

#TODO: Maybe add sensor data encode to this file

class HolooceanInterface():
    '''
    Class for abstracting holoocean python interface
    Lists sensors and formats sensor data
    '''

    def __init__(self, scenario_path):
        """
        Initialize holoocean enviornment with a path to a json file for the scenario
        Create the vehicle object and the dynamics object
        """
        
        file_path = Path(scenario_path)
        scenario = None

        with file_path.open() as params_file:
            scenario = json.load(params_file)
        
        #TODO: make sure dynamics sensor is enabled 
        self.env = holoocean.make(scenario_cfg=scenario,) #show_viewport=False)
        self.scenario = scenario

        self.sensors = self.create_sensor_list()


    def create_sensor_list(self):
        scenario = self.scenario

        if len(scenario["agents"]) > 1:
            self.multi_agent_scenario = True
            print("Give sensors unique names that are reported on multiple agents")
        else:
            self.multi_agent_scenario = False

        sensors = []

        for agent in scenario["agents"]:
            #For each agent the control surface commands can be published 
            #TODO: Handle Multi agent scenario here
            if "publish_commands" in agent and agent["publish_commands"]==True:
                encoder_class = encoders.get("ControlCommand")
                config = {}
                config['sensor_name'] = "ControlCommand"
                config['sensor_type'] = "ControlCommand"
                config['agent_name'] = agent['agent_name']
                sensors.append(encoder_class(config))

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

                            sensors.append(encoder_class(sensor_copy))
                    else:
                        sensor['agent_name'] = agent['agent_name']
                        encoder = encoders[sensor['sensor_type']]
                        sensors.append(encoder(sensor))
        
        return sensors

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
                print(f"Error processing sensor: {sensor.name}, type: {sensor.type}, error: {str(e)}")

    def tick(self, command):
        """
        Step the holoocean enviornment and the vehicle dynamics 
        Return the state
        """
        state = self.env.step(command) #To publish data to ros correctly, we should only tick the enviornment once each step

        return state
    
    def get_scenario(self):
        return self.env._scenario
    
    def get_tick_rate(self):
        return self.env._ticks_per_sec
    
    def get_frame_rate(self):
        return self.env._frames_per_sec
    
    def get_period(self):
        return 1.0/self.env._ticks_per_sec
    





