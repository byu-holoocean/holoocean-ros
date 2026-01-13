import numpy as np
import holoocean

from pathlib import Path
from holoocean_main.interface.sensor_data_encode import encoders, multi_publisher_sensors
from holoocean.fossen_dynamics.fossen_interface import FossenInterface, get_vehicle_model


class HolooceanInterface():
    '''
    Class for abstracting holoocean python interface
    Lists sensors and formats sensor data
    '''

    def __init__(self, scenario_path, node=None, show_viewport=True, publish_commands=True, arrow_flag=True, render_quality=None):
        """
        Initialize holoocean enviornment with a path to a json file for the scenario
        Create the vehicle object and the dynamics object
        """
        # Class Variables
        self.fossen_agents = []
        self.sensors = []
        self.node = node
        self.publish_commands = publish_commands
        self.arrow_flag = arrow_flag

        # TODO error handling to make sure its a json format?
        scenario_path = Path(scenario_path)
        scenario = holoocean.packagemanager.load_scenario_file(scenario_path)
        # TODO update holoocean to acccepth the scenario file path
        self.env = holoocean.make(scenario_cfg=scenario, show_viewport=show_viewport)
        self.state = self.env.tick()
        self.scenario = self.env._scenario
        self.render_quality = render_quality
        self.set_render_quality()
        
        self.parse_scenario()
        
        self.create_sensor_list()
        self.create_publishers()

        self.create_agent_command_buffer()

          
    def set_render_quality(self, value=None):
        if value is not None:
            self.render_quality = value

        if self.render_quality is not None:
            self.env.set_render_quality(self.render_quality)

    def create_agent_command_buffer(self):
        self.agent_commands = {}
        for agent_name in self.env.agents:
            agent = self.env.agents[agent_name] 
            print(agent_name, agent.action_space._shape)
            self.agent_commands[agent_name] = np.zeros(agent.action_space._shape, float)

    def initialize_fossen(self):
        # TODO this could be simplified but on the holoocean side
        # By creating the fossen models on the holoocean side
        self.fossen_agents = [agent["agent_name"] for agent in self.scenario.get("agents", []) if "fossen_model" in agent]
        self.fossen = FossenInterface(self.fossen_agents, self.scenario)


    def check_multi_agent(self):
        self.multi_agent_scenario = len(self.scenario.get("agents", [])) > 1

    def get_agent_state(self, agent_name):
        if self.multi_agent_scenario:
            return self.state[agent_name]
        else:
            return self.state

    def parse_scenario(self):
        self.check_multi_agent()
        self.initialize_fossen()

    def create_sensor_list(self):
        scenario = self.scenario

        for agent in scenario["agents"]:
            #For each agent the control surface commands can be published 
            agent_name = agent['agent_name']
            if self.publish_commands and (agent_name in self.fossen_agents):
                encoder_class = encoders.get("ControlCommand")
                config = {}
                config['sensor_name'] = "ControlCommand"
                config['sensor_type'] = "ControlCommand"
                config['agent_name'] = agent_name
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
                agent_state = self.get_agent_state(sensor.agent_name)
                msg = sensor.encode(agent_state[sensor.state_name])

                # Header
                msg.header.stamp.sec = int(state['t'])  # Set seconds part from state['t']
                msg.header.stamp.nanosec = int((state['t'] - msg.header.stamp.sec) * 1e9)

                sensor.publisher.publish(msg)
            except KeyError:
                # Handle the case where sensor data is not available
                # This is a common case where sensor data rates are configured to not be every tick
                pass
            except Exception as e:
                # Handle other exceptions
                print(f"Error processing sensor: {sensor.name}, type: {sensor.type}, error: {str(e)}")

    def tick(self):
        """
        Step the holoocean enviornment and the vehicle dynamics 
        Return the state
        """
        
        # for agent in self.scenario.get("agents", []):
        # TODO this should work with spawned agents. Need to double check
        for agent_name in self.env.agents:
            agent = self.env.agents[agent_name]

            if agent_name in self.fossen_agents:
                command = self.fossen.update(agent_name, self.state) #Calculate accelerations to be applied to HoloOcean agent
                # Draw the autopilot arrows
                self.draw_arrow(agent_name)
            else:
                # This is the case for the general agent without fossen Dynamics
                command = self.agent_commands[agent_name]

            self.env.act(agent_name, command)

            

        self.state = self.env.tick() #To publish data to ros correctly, we should only tick the enviornment once each step


        # Upadate the Control command for publishing for each fossen agent
        # TODO figure out and document what order these come in
        for agent in self.fossen_agents:
            u_control = self.fossen.get_u_control(agent)

            if self.multi_agent_scenario:
                self.state[agent]["ControlCommand"] = np.array(u_control)
            else:
                self.state["ControlCommand"] = np.array(u_control)
                
        self.publish_sensor_data()

        time = self.state['t']

        return time

    # TODO should be able to get rid of the if statement here since the enviornment should be initialized always with time figured out
    def get_scenario(self):
        return self.env._scenario
    
    def get_tick_rate(self):
        if self.env._ticks_per_sec is not None:
            return self.env._ticks_per_sec
        else:
            if "ticks_per_sec" in self.scenario:
                return self.scenario['ticks_per_sec']
            else:
                ValueError('ticks_per_sec not specified in scenario')
    
    def get_period(self):
        return 1.0/self.get_tick_rate()

    def set_control_mode(self, vehicle_name, mode):
        if not self.check_fossen_agent(vehicle_name, 'Set Control Mode'):
            return
        self.fossen.set_control_mode(vehicle_name, mode)

    def check_fossen_agent(self, vehicle_name, command_info):
        '''
        Raises:
        - KeyError: If the vehicle name is not found in the vehicle registry.
        '''
        # Check if the vehicle exists
        if vehicle_name not in self.fossen_agents:
            print(f"WARNING: Vehicle '{vehicle_name}' not found in the fossen vehicle registry. Cannot use command {command_info}")
            return False
        else:
            return True
    
    def set_u_control(self, vehicle_name, u_control):
        """
        Set the u_command vector for a specific vehicle.

        Parameters:
        - vehicle_name (str): The name of the vehicle to control.
        - u_control (list or array-like): The control input vector to set.

        - ValueError: If the dimension of u_control does not match the vehicle's expected dimension.
        """
        if not self.check_fossen_agent(vehicle_name, 'Set U Control'):
            return

        vehicle = self.fossen.vehicles[vehicle_name]

        # Check that the control vector matches the vehicle's expected input dimension
        if len(u_control) != vehicle.dimU:
            raise ValueError(
                f"Control vector length {len(u_control)} does not match expected dimension {vehicle.dimU} for vehicle '{vehicle_name}'."
            )

        # Assign the control command
        self.fossen.set_u_control(vehicle_name, u_control) 
    
    def set_agent_command(self, vehicle, command):
        # TODO probably not the best way to do this but will work for now
        self.agent_commands[vehicle] = command


    def reset_enviornment(self, vehicle=None):
        if vehicle is None:
            self.state = self.env.reset()

        self.set_render_quality()

    # TODO add error handling for frame_id and that agent has that type of goal
    def set_depth(self, vehicle_name, depth):
        if not self.check_fossen_agent(vehicle_name, 'Set Depth'):
            return
        self.fossen.set_depth_goal(vehicle_name, depth)

    def set_heading(self, vehicle_name, heading):
        if not self.check_fossen_agent(vehicle_name, 'Set Heading'):
            return
        self.fossen.set_heading_goal(vehicle_name, heading)

    def set_speed(self, vehicle_name, speed):
        if not self.check_fossen_agent(vehicle_name, 'Set Speed'):
            return
        self.fossen.set_rpm_goal(vehicle_name, int(speed))

    
    def draw_arrow(self, agent_name):
        if self.arrow_flag == False:
            return
        
        agent_state = self.get_agent_state(agent_name)

        # TODO try execpt on if there is no dynamics sensor
        pos = agent_state['DynamicsSensor'][6:9]  # [x, y, z]

        ### DEPTH ARROW ###
        # TODO check if the depth autopilot is used.
        # For plotting HSD and arrows 
        depth = self.fossen.vehicles[agent_name].ref_z

        #change color if within 2 meters
        if abs(depth + pos[2]) <= 2.0:
            color = [0,255,0]
        else:
            color = [255,0,0]

        self.env.draw_arrow(pos.tolist(), end=[pos[0], pos[1], -depth], color=color, thickness=5, lifetime=self.get_period()+0.01)
        
        
        ### HEADING ARROW ###
        # TODO check if heading autopilot is used 
        # TODO change the color of the heading arrow if within tolerance

        heading = self.fossen.vehicles[agent_name].ref_psi
        heading_rad = np.deg2rad(heading)

        x_end = pos[0] + 3 * np.cos(heading_rad)
        y_end = pos[1] - 3 * np.sin(heading_rad)
        self.env.draw_arrow(pos.tolist(), end=[x_end, y_end, pos[2]], color=[0,0,255], thickness=10, lifetime=self.get_period()+0.01)

    def draw_debug_points(self, points, colors, thickness, lifetime):
        # points: list of [x, y, z]
        # color: [r, g, b] (0-255)
        for i, point in enumerate(points):        # Issue while developing, unable to pass list of points, so drawing one at a time
            color = colors[i]
            self.env.draw_point(
                loc=point,
                color=color,
                thickness=thickness,
                lifetime=lifetime
            )

