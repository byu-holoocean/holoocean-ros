import numpy as np
import holoocean
from holoocean.vehicle_dynamics import *
from holoocean.dynamics import *
from pathlib import Path
import json

import time

class VehicleInterface():
    '''
    Class for abstracting simulation updates
    Interface for controlling the vehicle
    Pass in Depth, heading, speed 
    Return the state
    '''

    def __init__(self, scenario_path):
        """
        Initialize holoocean enviornment with a path to a json file for the scenario
        Create the vehicle object and the dynamics object
        """
        
        file_path = Path(scenario_path)
        scenario = None
        self.draw = False
        self.time_warp = 1.0

        with file_path.open() as params_file:
            scenario = json.load(params_file)

        if "draw_arrow" in scenario:
            self.draw = scenario["draw_arrow"]
        
        #TODO: make sure dynamics sensor is enabled 
        self.past = time.time()
        self.env = holoocean.make(scenario_cfg=scenario,) #show_viewport=False)
        self.scenario = scenario

        #If set to max speed time warp will be 0 because frames per sec will be 0.
        #Time warp is limited to computer possiblity. TODO: use time to make sure it is ticking at right speeds
        self.time_warp = self.env._frames_per_sec / self.env._ticks_per_sec

        self.accel = np.array(np.zeros(6),float)

        #Create vehicle object attached to holoocean agent with dynamic parameters 
        #TODO: Change the vehicle that is being setup from the parameters
        self.vehicle = threeFinInd(scenario, 'auv0','depthHeadingAutopilot')
        # self.vehicle.set_control_mode('depthHeadingAutopilot') #In this mode PID controller calculates control commands (u_control)    

        #Create dynamics object passing in the vehicle created
        print('Ticks per sec: ', self.env._ticks_per_sec)
        print("Time Warp:", self.time_warp)
        self.torpedo_dynamics = FossenDynamics(self.vehicle,self.env._ticks_per_sec)  


    def send_command(self, depth, heading, rpm, draw=False):
        """
        Set desired depth, heading, rpm (float)
        Coordinate system NED. Units meters, degrees, meters/sec        
        """
        if draw:
            self.draw = draw
            print("Drawing Arrow for Depth Heading")
        self.vehicle.set_goal(depth, heading, rpm)     #Changes depth (positive depth), heading, thruster RPM goals for controller

    def tick(self):
        """
        Step the holoocena enviornment and the vehicle dynamics 
        Return the state
        """
        state = self.env.step(self.accel) #To publish data to ros correctly, we should only tick the enviornment once each step

        self.accel = self.torpedo_dynamics.update(state) #Calculate accelerations to be applied to HoloOcean agent

        #TODO: Handle the multi agent case for the control commands here
        fins = np.rad2deg(self.torpedo_dynamics.u_actual[:-1])
        thruster = self.torpedo_dynamics.u_actual[-1]

        state["ControlCommand"] = np.append(fins,thruster)

        if self.draw:
            self.draw_arrow(state)

        return state
    
    def draw_arrow(self, state):
        # For plotting and arrows 
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

        self.env.draw_arrow(pos.tolist(), end=[x_end, y_end, pos[2]], color=[0,0,255], thickness=5, lifetime=self.get_period()+0.01)
        self.env.draw_arrow(pos.tolist(), end=[pos[0], pos[1], -depth], color=color, thickness=5, lifetime=self.get_period()+0.01)
    

    def get_scenario(self):
        return self.env._scenario
    
    def get_tick_rate(self):
        return self.env._ticks_per_sec
    
    def get_period(self):
        return 1.0/self.env._ticks_per_sec
    
    def get_time_warp(self):
        return self.time_warp
    
    def get_warp_period(self):
        return self.get_period() / self.get_time_warp()




