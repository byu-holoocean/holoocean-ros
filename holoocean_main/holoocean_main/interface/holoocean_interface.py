import numpy as np
import holoocean

from pathlib import Path
from holoocean_main.interface.sensor_data_encode import encoders, multi_publisher_sensors
from holoocean.fossen_interface import FossenInterface, get_vehicle_model

#TODO: Maybe add sensor data encode to this file

class HolooceanInterface():
    '''
    Class for abstracting holoocean python interface
    Lists sensors and formats sensor data
    '''

    def __init__(self, scenario_path, node=None):
        """
        Initialize holoocean enviornment with a path to a json file for the scenario
        Create the vehicle object and the dynamics object
        """
        # Class Variables
        self.fossen_agents = []
        self.sensors = []
        self.node = node

        # TODO error handling to make sure its a json format?
        scenario_path = Path(scenario_path)
        scenario = holoocean.packagemanager.load_scenario_file(scenario_path)
        # TODO update holoocean to acccepth the scenario file path
        self.env = holoocean.make(scenario_cfg=scenario)
        self.state = self.env.tick()
        self.scenario = self.env._scenario
        
        self.parse_scenario()
        
        # TODO fix initialized
        self.initialized = True
        self.create_sensor_list()
        self.create_publishers()
        

        

    def initialize_fossen(self):
        # TODO this could be simplified but on the holoocean side
        # By creating the fossen models on the holoocean side
        self.fossen_agents = [agent["agent_name"] for agent in self.scenario.get("agents", []) if "fossen_model" in agent]
        self.fossen = FossenInterface(self.fossen_agents, self.scenario)


    def check_multi_agent(self):
        self.multi_agent_scenario = len(self.scenario.get("agents", [])) > 1

    def parse_scenario(self):
        self.check_multi_agent()
        self.initialize_fossen()


    def create_sensor_list(self):
        scenario = self.scenario

        for agent in scenario["agents"]:
            #For each agent the control surface commands can be published 
            #TODO: Handle Multi agent scenario here
            # TODO move publish commands to the yaml ros params
            if "publish_commands" in agent and agent["publish_commands"]==True:
                encoder_class = encoders.get("ControlCommand")
                config = {}
                config['sensor_name'] = "ControlCommand"
                config['sensor_type'] = "ControlCommand"
                config['agent_name'] = agent['agent_name']
                config['state_name'] = 'ControlCommand'
                self.sensors.append(encoder_class(config))

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
                            self.sensors.append(encoder_class(sensor_copy))
                    else:
                        sensor_copy = sensor.copy()  # Create a copy of the sensor dictionary
                        sensor_copy['agent_name'] = agent['agent_name']
                        sensor_copy['state_name'] = sensor['sensor_name']  # Need the name to pull the data out of the state
                        encoder = encoders[sensor['sensor_type']]
                        self.sensors.append(encoder(sensor_copy))
        
    
    def create_publishers(self):
        for sensor in self.sensors:
            topic_name = sensor.agent_name + '/' + sensor.name
            sensor.publisher = self.node.create_publisher(sensor.message_type, topic_name, 10)

    def publish_sensor_data(self):
        # TODO check if i should copy here
        state = self.state.copy()
        for sensor in self.sensors:
            try:
                if self.multi_agent_scenario:
                    msg = sensor.encode(state[sensor.agent_name][sensor.state_name])

                else:
                    msg = sensor.encode(state[sensor.state_name])

                # Header
                # TODO publish sim time on clock 
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
        
        for agent in self.scenario.get("agents", []):
            agent_name = agent["agent_name"]

            # TODO work here for non fossen agents
            command = np.zeros(6 ,float)

            if agent_name in self.fossen_agents:
                command = self.fossen.update(agent_name, self.state) #Calculate accelerations to be applied to HoloOcean agent

            self.env.act(agent_name, command)

        self.state = self.env.tick() #To publish data to ros correctly, we should only tick the enviornment once each step

        self.publish_sensor_data()


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


    def set_u_command_callback(self, msg):
        # TODO fix this and do error handling when i fix the ucommand message
        vehicle = self.fossen.vehicles[msg.header.frame_id]
        u_control = np.zeros(vehicle.dimU, np.float64)
        for i in range(vehicle.dimU - 1):
            u_control[i] = msg.fin[i]
            # print(i, u_control[i])
        
        u_control = np.deg2rad(u_control)
        
        u_control[-1] = float(msg.thruster)

        self.fossen.set_u_control_rad(self.main_agent, u_control)     

    # TODO add error handling for frame_id and that agent has that type of goal
    def depth_callback(self, msg):
        self.fossen.set_depth_goal(msg.header.frame_id, msg.data)

    def heading_callback(self, msg):
        self.fossen.set_heading_goal(msg.header.frame_id, msg.data)

    def speed_callback(self, msg):
        self.fossen.set_rpm_goal(msg.header.frame_id, int(msg.data))

