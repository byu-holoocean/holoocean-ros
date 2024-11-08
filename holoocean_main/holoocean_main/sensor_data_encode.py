from abc import ABC, abstractmethod
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from holoocean_interfaces.msg import DVLSensorRange, UCommand
import numpy as np


multi_publisher_sensors = {
    'DVLSensor': ['Velocity', 'Range'],
    'DynamicsSensor': ['Odom', 'IMU']
}

class SensorPublisher(ABC):
    def __init__(self, sensor_dict):
        self.name = sensor_dict['sensor_name']
        self.type = sensor_dict['sensor_type']
        self.agent_name = sensor_dict['agent_name']
        self.state_name = sensor_dict['state_name']
        if "configuration" in sensor_dict:
            self.config = sensor_dict['configuration']
        else:
            self.config = None
        self.publisher = None


    @abstractmethod
    def encode(self, sensor_data):
        pass

class IMUEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = Imu
        

        self.accel_cov = [0.0] * 9
        self.ang_cov = [0.0] * 9

        if self.config is not None:
            if 'AccelCov' in self.config:
                if isinstance(self.config['AccelCov'][0], list):
                    flattened_cov = [item for sublist in self.config['AccelCov'] for item in sublist]
                    if len(flattened_cov) == 9:
                        self.accel_cov = [float(value) for value in flattened_cov]                   
                elif len(self.config['AccelCov']) == 3:
                    self.accel_cov[0] = float(self.config['AccelCov'][0])
                    self.accel_cov[4] = float(self.config['AccelCov'][1])
                    self.accel_cov[8] = float(self.config['AccelCov'][2])
                else:
                    raise ValueError("AccelCov must be a list of length 3 or 3x3.")
            
            if 'AngVelCov' in self.config:
                if isinstance(self.config['AngVelCov'][0], list):
                    flattened_cov = [item for sublist in self.config['AngVelCov'] for item in sublist]
                    if len(flattened_cov) == 9:
                        self.ang_cov = [float(value) for value in flattened_cov]                   
                elif len(self.config['AngVelCov']) == 3:
                    self.ang_cov[0] = float(self.config['AngVelCov'][0])
                    self.ang_cov[4] = float(self.config['AngVelCov'][1])
                    self.ang_cov[8] = float(self.config['AngVelCov'][2])
                else:
                    raise ValueError("AngVelCov must be a list of length 3 or 3x3.")
           
    
    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = 'odom'
        msg.orientation_covariance[0] = -1

        # Assign acceleration
        msg.linear_acceleration.x = float(sensor_data[0, 0])
        msg.linear_acceleration.y = float(sensor_data[0, 1])
        msg.linear_acceleration.z = float(sensor_data[0, 2])

        # Assign angular velocity
        msg.angular_velocity.x = float(sensor_data[1, 0])
        msg.angular_velocity.y = float(sensor_data[1, 1])
        msg.angular_velocity.z = float(sensor_data[1, 2])

        
        msg.linear_acceleration_covariance = self.accel_cov
        msg.angular_velocity_covariance = self.ang_cov

        return msg

class DVLEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)
        
        self.message_type = TwistWithCovarianceStamped

        self.cov = [0.0] * 36

        #TODO: Holoocean Sensor sets covariance on each beam velocity lenght 4

        if self.config is not None:
            if 'VelCov' in self.config:
                if isinstance(self.config['VelCov'][0], list):
                    flattened_cov = [item for sublist in self.config['VelCov'] for item in sublist]
                    self.cov[0] = float(flattened_cov[0])
                    self.cov[7] = float(flattened_cov[5])
                    self.cov[14] = float(flattened_cov[10])                   
                elif len(self.config['VelCov']) == 4:
                    self.cov[0] = float(self.config['VelCov'][0])
                    self.cov[7] = float(self.config['VelCov'][1])
                    self.cov[14] = float(self.config['VelCov'][2])
                else:
                    raise ValueError("VelCov must be a list of length 4 or 4x4.")

        

    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = 'odom'
        # Assign velocity
        msg.twist.twist.linear.x = float(sensor_data[0])
        msg.twist.twist.linear.y = float(sensor_data[1])
        msg.twist.twist.linear.z = float(sensor_data[2])

        msg.twist.covariance = self.cov

        return msg

class DVLRangeEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = DVLSensorRange


    def encode(self, sensor_data):
        msg = self.message_type()

        msg.range[0] = float(sensor_data[3])
        msg.range[1] = float(sensor_data[4])
        msg.range[2] = float(sensor_data[5])
        msg.range[3] = float(sensor_data[6])

        return msg

class DepthEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = PoseWithCovarianceStamped
        self.cov = [0.0] * 36

        if self.config is not None:
            if 'Cov' in self.config:
                self.cov[14] = float(self.config['Cov'])

    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.z = float(sensor_data[0])
        msg.pose.covariance = self.cov
        return msg

class LocationEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = PoseWithCovarianceStamped

        self.cov = [0.0] * 36

        if self.config is not None:
            if 'Cov' in self.config:
                if isinstance(self.config['Cov'][0], list):
                    flattened_cov = [item for sublist in self.config['Cov'] for item in sublist]
                    self.cov[0] = float(flattened_cov[0])
                    self.cov[7] = float(flattened_cov[5])
                    self.cov[14] = float(flattened_cov[10])                   
                elif len(self.config['Cov']) == 3:
                    self.cov[0] = float(self.config['Cov'][0])
                    self.cov[7] = float(self.config['Cov'][1])
                    self.cov[14] = float(self.config['Cov'][2])
                else:
                    raise ValueError("Cov must be a list of length 3 or 3x3.")

    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = 'odom'
        #Frame ID might be map
        msg.pose.pose.position.x = float(sensor_data[0])
        msg.pose.pose.position.y = float(sensor_data[1])
        msg.pose.pose.position.z = float(sensor_data[2])
        msg.pose.covariance = self.cov
        return msg

class RotationEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)
        
        self.message_type = Vector3Stamped


    def encode(self, sensor_data):
        rpy_msg = self.message_type()
        rpy_msg.vector.x = float(sensor_data[0])
        rpy_msg.vector.y = float(sensor_data[1])
        rpy_msg.vector.z = float(sensor_data[2])
        return rpy_msg

class VelocityEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)
        
        self.message_type = TwistWithCovarianceStamped


    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = 'odom'
        #Frame id might actually be base link for velocity

        # Assign velocity
        msg.twist.twist.linear.x = float(sensor_data[0])
        msg.twist.twist.linear.y = float(sensor_data[1])
        msg.twist.twist.linear.z = float(sensor_data[2])

        return msg

class DynamicsEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)
        
        self.message_type = Odometry


    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'odom'
        if len(sensor_data) == 18:
            sensor_data.append(-100) # Should error out if mistakenly trying to use it as a quaternion
        elif len(sensor_data) != 19:
            raise TypeError("Dynamics data is not the expected shape for ROS publishing")

        msg.twist.twist.linear.x = float(sensor_data[3])
        msg.twist.twist.linear.y = float(sensor_data[4])
        msg.twist.twist.linear.z = float(sensor_data[5])

        msg.pose.pose.position.x = float(sensor_data[6])
        msg.pose.pose.position.y = float(sensor_data[7])
        msg.pose.pose.position.z = float(sensor_data[8])

        msg.twist.twist.angular.x = float(sensor_data[12])
        msg.twist.twist.angular.y = float(sensor_data[13])
        msg.twist.twist.angular.z = float(sensor_data[14])

        msg.pose.pose.orientation.x = float(sensor_data[15])
        msg.pose.pose.orientation.y = float(sensor_data[16])
        msg.pose.pose.orientation.z = float(sensor_data[17])
        msg.pose.pose.orientation.w = float(sensor_data[18])

        msg.pose.covariance = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        return msg

class DynamicsIMUEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)
        
        self.message_type = Imu

        self.use_covariance = True
         # Define arbitrary IMU covariance matrices
        self.orientation_covariance = np.array([
            [0.01, 0, 0],
            [0, 0.01, 0],
            [0, 0, 0.01]
        ])

        self.angular_velocity_covariance = np.array([
            [0.01, 0, 0],
            [0, 0.01, 0],
            [0, 0, 0.01]
        ])

        self.linear_acceleration_covariance = np.array([
            [0.1, 0, 0],
            [0, 0.1, 0],
            [0, 0, 0.1]
        ])


    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = 'base_link'

        # Orientation Quaternion
        msg.orientation.x = float(sensor_data[15])
        msg.orientation.y = float(sensor_data[16])
        msg.orientation.z = float(sensor_data[17])
        msg.orientation.w = float(sensor_data[18])

        # Assign acceleration
        msg.linear_acceleration.x = float(sensor_data[0])
        msg.linear_acceleration.y = float(sensor_data[1])
        msg.linear_acceleration.z = float(sensor_data[2])

        # Assign angular velocity
        msg.angular_velocity.x = float(sensor_data[9])
        msg.angular_velocity.y = float(sensor_data[10])
        msg.angular_velocity.z = float(sensor_data[11])

        if self.use_covariance:
            msg.orientation_covariance = self.orientation_covariance.flatten().tolist()
            msg.angular_velocity_covariance = self.angular_velocity_covariance.flatten().tolist()
            msg.linear_acceleration_covariance = self.linear_acceleration_covariance.flatten().tolist()

        return msg

class GPSEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)
        
        self.message_type = Odometry


        self.cov = [0.0] * 36

        if self.config is not None:
            if 'Cov' in self.config:
                if isinstance(self.config['Cov'][0], list):
                    flattened_cov = [item for sublist in self.config['Cov'] for item in sublist]
                    self.cov[0] = float(flattened_cov[0])
                    self.cov[7] = float(flattened_cov[5])
                    self.cov[14] = float(flattened_cov[10])                   
                elif len(self.config['Cov']) == 3:
                    self.cov[0] = float(self.config['Cov'][0])
                    self.cov[7] = float(self.config['Cov'][1])
                    self.cov[14] = float(self.config['Cov'][2])
                else:
                    raise ValueError("Cov must be a list of length 3 or 3x3.")

    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(sensor_data[0])
        msg.pose.pose.position.y = float(sensor_data[1])
        msg.pose.pose.position.z = float(sensor_data[2])
        msg.pose.covariance = self.cov
        return msg

class CommandEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = UCommand
        self.fin = [360.0] * 4


    def encode(self, sensor_data):
        msg = self.message_type()
        msg.fin = self.fin

        #Control commands should be in a list with fins first and thruster last value in list (max 4 fins)
        fin_count = len(sensor_data) - 1

        for i in range(fin_count):
            msg.fin[i] = float(sensor_data[i])
        
        msg.thruster = int(sensor_data[-1])

        return msg

# Define other encoders similarly...


encoders = {
    'IMUSensor': IMUEncoder,
    'DVLSensorVelocity': DVLEncoder,
    'DVLSensorRange': DVLRangeEncoder,
    'DepthSensor': DepthEncoder,
    'LocationSensor': LocationEncoder,
    'RotationSensor': RotationEncoder,
    'VelocitySensor': VelocityEncoder,
    'DynamicsSensorOdom': DynamicsEncoder,
    'DynamicsSensorIMU': DynamicsIMUEncoder,
    'GPSSensor': GPSEncoder,
    'ControlCommand': CommandEncoder,
    # Add other sensor type encoders here...
}