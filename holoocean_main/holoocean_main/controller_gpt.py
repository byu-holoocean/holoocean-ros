import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, Vector3Stamped
from holoocean_interfaces.msg import UCommand
import numpy as np

from pathlib import Path
import json

from holoocean.dynamics_controls.control import integralSMC
from holoocean.dynamics_controls.gnc import ssa

class ControllerExample(Node):

    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('params_file', '')
        
        file_path = Path(self.get_parameter('params_file').get_parameter_value().string_value)

        scenario = None

        with file_path.open() as params_file:
            scenario = json.load(params_file)
        
        self.time_warp = 1.0
        if "frames_per_sec" in scenario:
            self.time_warp = scenario["frames_per_sec"]/ scenario["ticks_per_sec"]

        print("Time Warp:", self.time_warp)

        ############ HSD subscribers: ############
        self.depth = 0.0
        self.heading = 0.0
        self.speed = 0.0

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

        # Speed in x body frame (forward, m/s)
        self.speed_sub = self.create_subscription(
            Float64,
            'speed',
            self.speed_callback,
            10
        )

        ############ Data Subscriber ############
        self.nu = np.zeros(3, np.float64)
        self.roll = 0.0
        self.pitch = 0.0 
        self.yaw = 0.0
        self.z = 0.0

        #DVL Sensor returns velocity in body frame
        self.vel_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            'DVLSensorVelocity',
            self.vel_callback,
            10)

        #Roation Sensor return rotation 
        self.rotation_sub = self.create_subscription(
            Vector3Stamped,
            'RotationSensor',
            self.rotation_callback,
            10)

        self.depth_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'DepthSensor',
            self.depth_callback,
            10)
        
        ######## Control Surfaces Publishers #########

        self.publisher = self.self.create_publisher(UCommand, 'u_command', 10)

        ######## Control loop timer ###############
        if self.time_warp > 0:
            self.timer_period =   0.1 / self.time_warp  # seconds
            timer_publish_period = 0.5 / self.time_warp  # seconds
            self.timer_control = self.create_timer(self.timer_period, self.control_loop)
        else:
            ValueError("frames_per_sec cannot be 0 for time warping. Set a value > 0 ")

        autopilot = scenario.get("autopilot", {})
        depth_params = autopilot.get("depth", {})
        heading_params = autopilot.get("heading", {})
        surge_params = autopilot.get("surge", {})

        self.dynamic_parameters = {
            "mass": scenario.get("mass", 16),
            "length": scenario.get("length", 1.6),
            "rho": scenario.get("rho", 1026),
            "diam": scenario.get("diam", 0.19),
            "r_bg": scenario.get("r_bg", [0, 0, 0.02]),
            "r_bb": scenario.get("r_bb", [0, 0, 0]),
            "r44": scenario.get("r44", 0.3),
            "Cd": scenario.get("Cd", 0.42),
            "T_surge": scenario.get("T_surge", 20),
            "T_sway": scenario.get("T_sway", 20),
            "zeta_roll": scenario.get("zeta_roll", 0.3),
            "zeta_pitch": scenario.get("zeta_pitch", 0.8),
            "T_yaw": scenario.get("T_yaw", 1),
            "K_nomoto": scenario.get("K_nomoto", 5.0 / 20.0)
        }

        self.autopilot_parameters = {
            'depth': {
                'wn_d_z': depth_params.get('wn_d_z', 0.2),
                'Kp_z': depth_params.get('Kp_z', 0.1),
                'T_z': depth_params.get('T_z', 100),
                'Kp_theta': depth_params.get('Kp_theta', 5.0),
                'Kd_theta': depth_params.get('Kd_theta', 2.0),
                'Ki_theta': depth_params.get('Ki_theta', 0.3),
                'K_w': depth_params.get('K_w', 5.0),
                'theta_max_deg': depth_params.get('theta_max_deg', 30),
            },
            'heading': {
                'wn_d': heading_params.get('wn_d', 1.2),
                'zeta_d': heading_params.get('zeta_d', 0.8),
                'r_max': heading_params.get('r_max', 0.9),
                'lam': heading_params.get('lam', 0.1),
                'phi_b': heading_params.get('phi_b', 0.1),
                'K_d': heading_params.get('K_d', 0.5),
                'K_sigma': heading_params.get('K_sigma', 0.05),
            },
            'surge': {
                'kp_surge': surge_params.get('kp_surge', 400.0),
                'ki_surge': surge_params.get('ki_surge', 50.0),
                'kd_surge': surge_params.get('kd_surge', 30.0),
            }
        }

        # Heading 
        self.psi_d = 0.0
        self.r_d = 0
        self.a_d = 0
        self.e_psi_int = 0

        self.wn_d = self.autopilot_parameters["heading"]["wn_d"]
        self.zeta_d = self.autopilot_parameters["heading"]["zeta_d"]
        self.K_d = self.autopilot_parameters["heading"]["K_d"]
        self.K_sigma = self.autopilot_parameters["heading"]["K_sigma"]
        self.lam = self.autopilot_parameters["heading"]["lam"]
        self.phi_b = self.autopilot_parameters["heading"]["phi_b"]
        self.r_max = self.autopilot_parameters["heading"]["r_max"]
        self.T_nomoto = self.dynamic_parameters["T_yaw"]
        self.K_nomoto = self.dynamic_parameters["K_nomoto"]

        # Speed - Example values; replace with values from the scenario if needed

        # Depth - Example values; replace with values from the scenario if needed
        self.init_depth = False
        self.z_int = 0.0
        self.wn_d_z = self.autopilot_parameters["depth"]["wn_d_z"]
        self.Kp_z = self.autopilot_parameters["depth"]["Kp_z"]
        self.T_z = self.autopilot_parameters["depth"]["T_z"]
        self.theta_max = self.autopilot_parameters["depth"]["theta_max_deg"]
        self.Kp_theta = self.autopilot_parameters["depth"]["Kp_theta"]
        self.Kd_theta = self.autopilot_parameters["depth"]["Kd_theta"]
        self.Ki_theta = self.autopilot_parameters["depth"]["Ki_theta"]
        self.theta_int = self.autopilot_parameters["depth"]["theta_int"]
        self.K_w = self.autopilot_parameters["depth"]["K_w"]


        

    ###### Control Loop #########
    #TODO: Check the timing and make sure the control loop runs at correct hz
    #Consider making a flag to indicate that the data is correct

    #NEED ANGULAR VELOCITY data or numerically calculate pitch rate and yaw rate
    def control_loop(self):
        z = self.z
        theta = self.pitch              # pitch angle
        psi = self.yaw                # yaw angle
        w = self.nu[2]                   # heave velocity
        q = self.nu[4]                   # pitch rate
        r = self.nu[5]                   # yaw rate
        #Does the following line need ssa()?? #actually it comes from the smc calculator
        e_psi = psi - self.psi_d    # yaw angle tracking error
        e_r   = r - self.r_d        # yaw rate tracking error
        z_ref = self.depth          # heave position (depth) setpoint
        psi_ref = np.deg2rad(self.heading)   # yaw angle setpoint #TODO: check 

        if self.speed > 0:
            #######################################################################
            # Propeller command
            #######################################################################
            # TODO: Speed control loop here
            if self.speed_control:
                #TODO: Super of self?
                n = self.speedAutopilot(self.nu, self.timer_period)
            else:
                n = self.ref_n 
            
            #######################################################################            
            # Depth autopilot (succesive loop closure)
            #######################################################################
            # LP filtered desired depth command 
            if not self.init_depth:
                self.z_d = z    #On initialization of the autopilot the commanded depth is set to the current depth
                self.init_depth = True
            self.z_d  = np.exp( -self.timer_period * self.wn_d_z ) * self.z_d \
                + ( 1 - np.exp( -self.timer_period * self.wn_d_z) ) * z_ref  
                
            # PI controller    
            theta_d = self.Kp_z * ( (z - self.z_d) + (1/self.T_z) * self.z_int )

            if abs(theta_d) > self.theta_max:
                theta_d = np.sign(theta_d) * self.theta_max

            delta_s = -self.Kp_theta * ssa( theta - theta_d ) - self.Kd_theta * q \
                - self.Ki_theta * self.theta_int - self.K_w * w
            delta_sl = delta_sr = delta_s

            # Euler's integration method (k+1)
            self.z_int     += self.timer_period * ( z - self.z_d )
            self.theta_int += self.timer_period * ssa( theta - theta_d )

            #######################################################################
            # Heading autopilot (SMC controller)
            #######################################################################
            
            wn_d = self.wn_d            # reference model natural frequency
            zeta_d = self.zeta_d        # reference model relative damping factor


            # Integral SMC with 3rd-order reference model
            [delta_r, self.e_psi_int, self.psi_d, self.r_d, self.a_d] = \
                integralSMC( 
                    self.e_psi_int, 
                    e_psi, e_r, 
                    self.psi_d, 
                    self.r_d, 
                    self.a_d, 
                    self.T_nomoto, 
                    self.K_nomoto, 
                    wn_d, 
                    zeta_d, 
                    self.K_d, 
                    self.K_sigma, 
                    self.lam,
                    self.phi_b,
                    self.heading, 
                    self.r_max, 
                    self.timer_period 
                    )
                    
            # Euler's integration method (k+1)
            self.e_psi_int += self.timer_period * ssa( psi - self.psi_d )
            delta_rt = delta_rb = delta_r
            
            u_control = np.array([ delta_rt, delta_rb,delta_sl,delta_sr, n], float)

        else:
            u_control = np.array([ 0, 0,0,0, 0], float)

        return u_control

    
    #TODO: Seperate control loop into 3 different functions
    

    def depthHeadingAutopilot(self, eta, nu, sampleTime):
        """
        Returns:
            list:
                The control input u_control as a list: [delta_rt, delta_rb, delta_sl, delta_sr, n], where:

                - delta_rt: Rudder top angle (rad).
                - delta_rb: Rudder bottom angle (rad).
                - delta_sl: Stern left angle (rad).
                - delta_sr: Stern right angle (rad).
                - n: Propeller revolution (rpm).
        """

        z = eta[2]                  # heave position (depth)
        theta = eta[4]              # pitch angle
        psi = eta[5]                # yaw angle
        w = nu[2]                   # heave velocity
        q = nu[4]                   # pitch rate
        r = nu[5]                   # yaw rate
        e_psi = psi - self.psi_d    # yaw angle tracking error
        e_r   = r - self.r_d        # yaw rate tracking error
        z_ref = self.ref_z          # heave position (depth) setpoint
        psi_ref = self.ref_psi * self.D2R   # yaw angle setpoint
        
        if self.speed > 0:
            #######################################################################
            # Propeller command
            #######################################################################
            n = self.ref_n 
            
            #######################################################################            
            # Depth autopilot (succesive loop closure)
            #######################################################################
            # LP filtered desired depth command 
            if not self.init_depth:
                self.z_d = z    #On initialization of the autopilot the commanded depth is set to the current depth
                self.init_depth = True
            self.z_d  = np.exp( -sampleTime * self.wn_d_z ) * self.z_d \
                + ( 1 - np.exp( -sampleTime * self.wn_d_z) ) * z_ref  
                
            # PI controller    
            theta_d = self.Kp_z * ( (z - self.z_d) + (1/self.T_z) * self.z_int )

            if abs(theta_d) > self.theta_max:
                theta_d = np.sign(theta_d) * self.theta_max

            delta_s = -self.Kp_theta * ssa( theta - theta_d ) - self.Kd_theta * q \
                - self.Ki_theta * self.theta_int - self.K_w * w
            delta_sl = delta_sr = delta_s

            # Euler's integration method (k+1)
            self.z_int     += sampleTime * ( z - self.z_d )
            self.theta_int += sampleTime * ssa( theta - theta_d )

            #######################################################################
            # Heading autopilot (SMC controller)
            #######################################################################
            
            wn_d = self.wn_d            # reference model natural frequency
            zeta_d = self.zeta_d        # reference model relative damping factor


            # Integral SMC with 3rd-order reference model
            [delta_r, self.e_psi_int, self.psi_d, self.r_d, self.a_d] = \
                integralSMC( 
                    self.e_psi_int, 
                    e_psi, e_r, 
                    self.psi_d, 
                    self.r_d, 
                    self.a_d, 
                    self.T_nomoto, 
                    self.K_nomoto, 
                    wn_d, 
                    zeta_d, 
                    self.K_d, 
                    self.K_sigma, 
                    self.lam,
                    self.phi_b,
                    psi_ref, 
                    self.r_max, 
                    sampleTime 
                    )
                    
            # # Euler's integration method (k+1)
            # self.e_psi_int += sampleTime * ssa( psi - self.psi_d )
            delta_rt = delta_rb = delta_r
            
            u_control = np.array([ delta_rt, delta_rb,delta_sl,delta_sr, n], float)

        else:
            u_control = np.array([ 0, 0,0,0, 0], float)

        return u_control

    ### Data Sensor Callback ####
    def vel_callback(self, msg):
        self.nu[0] = msg.twist.twist.linear.x
        self.nu[1] = msg.twist.twist.linear.y
        self.nu[2] = msg.twist.twist.linear.z

    def rotation_callbacK(self, msg):
        self.roll = msg.vector.x
        self.pitch = msg.vector.y
        self.yaw = msg.vector.z

    def depth_callback(self, msg):
        self.z = msg.pose.pose.position.z

    ### HSD Callback Functions ###
    def depth_callback(self, msg):
        self.depth = msg.data

    def heading_callback(self, msg):
        self.heading = msg.data

    def speed_callback(self, msg):
        self.speed = msg.data


def main(args=None):
    rclpy.init(args=args)

    node = ControllerExample()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()