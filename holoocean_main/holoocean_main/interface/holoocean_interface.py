import numpy as np
import holoocean

from pathlib import Path
import json
from holoocean_main.interface.sensor_data_encode import encoders, multi_publisher_sensors

#TODO: Maybe add sensor data encode to this file

class HolooceanInterface():
    '''
    Class for abstracting holoocean python interface
    Lists sensors and formats sensor data
    '''

    def __init__(self, scenario_path, init=True, node=None):
        """
        Initialize holoocean enviornment with a path to a json file for the scenario
        Create the vehicle object and the dynamics object
        """
        self.node = node
        scenario_path = Path(scenario_path)
        # TODO error handling to make sure its a json format?
        scenario = holoocean.packagemanager.load_scenario_file(scenario_path)
        
        #TODO: Make a parameter to use the system time
        self.system_time = True
        
        #TODO: make sure dynamics sensor is enabled 
        if init:
            self.env = holoocean.make(scenario_cfg=scenario)
            # self.scenario = scenario
            self.scenario = self.env._scenario
            self.initialized = True
            self.sensors = self.create_sensor_list()
            self.create_publishers()
        else:
            self.scenario = scenario
            self.initialized = False



    def parse_scenario(self, path):
        file_path = Path(path)
        scenario = None

        with file_path.open() as params_file:
            scenario = json.load(params_file)

        return scenario
    

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
                config['state_name'] = 'ControlCommand'
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
                            sensor_copy['state_name'] = sensor['sensor_name']  # Need the name to pull the data out of the state
                            sensors.append(encoder_class(sensor_copy))
                    else:
                        sensor_copy = sensor.copy()  # Create a copy of the sensor dictionary
                        sensor_copy['agent_name'] = agent['agent_name']
                        sensor_copy['state_name'] = sensor['sensor_name']  # Need the name to pull the data out of the state
                        encoder = encoders[sensor['sensor_type']]
                        sensors.append(encoder(sensor_copy))
        
        return sensors
    
    def create_publishers(self):
        for sensor in self.sensors:
            sensor.publisher = self.node.create_publisher(sensor.message_type, sensor.name, 10)


    def publish_sensor_data(self, state):
        for sensor in self.sensors:
            try:
                if self.multi_agent_scenario:
                    msg = sensor.encode(state[sensor.agent_name][sensor.state_name])

                else:
                    msg = sensor.encode(state[sensor.state_name])

                # Header
                if self.system_time:
                    # TODO do sim time
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                else:
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

    def tick(self):
        """
        Step the holoocean enviornment and the vehicle dynamics 
        Return the state
        """
        command = self.fossen.update(self.main_agent, self.state) #Calculate accelerations to be applied to HoloOcean agent
        # TODO rework this to make easier for multi agent scenarios
        # TODO tick instead of step
        self.state = self.env.step(command) #To publish data to ros correctly, we should only tick the enviornment once each step

        self.publish_sensor_data(self.state)


        # TODO fix this and fix when its a publisher vs subscriber
        # #TODO: Handle the multi agent case for the control commands here
        # fins = np.rad2deg(self.torpedo_dynamics.u_actual[:-1])
        # thruster = self.torpedo_dynamics.u_actual[-1]

        # state["ControlCommand"] = np.append(fins,thruster)

    
    def get_scenario(self):
        if self.initialized:
            return self.env._scenario
        else:
            return self.scenario
    
    def get_tick_rate(self):
        if self.initialized:
            return self.env._ticks_per_sec
        else:
            if "ticks_per_sec" in self.scenario:
                return self.scenario['ticks_per_sec']
            else:
                ValueError('ticks_per_sec not specified in scenario')
    
    def get_frame_rate(self):
        if self.initialized:
            return self.env._frames_per_sec
        else:
            if "frames_per_sec" in self.scenario:
                return self.scenario['frames_per_sec']
            else:
                ValueError('frames_per_sec not specified in scenario')
    
    def get_period(self):
        return 1.0/self.get_tick_rate()
    
    def get_time_warp(self):
        #Check to make sure this is correct
        time_warp = self.get_frame_rate() / self.get_tick_rate()

        if time_warp <= 0:
            ValueError("frames_per_sec cannot be 0 for time warping. Set a value > 0 ")

        return time_warp

    def get_time_warp_period(self):
        return self.get_period() / self.get_time_warp()




