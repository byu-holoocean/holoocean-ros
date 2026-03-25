from abc import ABC, abstractmethod
from sensor_msgs.msg import Imu, Image, MagneticField, LaserScan, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from holoocean_interfaces.msg import DVLSensorRange, AgentCommand
from scipy.spatial.transform import Rotation
import numpy as np

PERFECT_COV = 1e-9
UNKNOWN_COV = -1

# TODO make a not about how the Dynamics Sensor IMU is not in local frame
multi_publisher_sensors = {
    'DVLSensor': ['Velocity', 'Range'],
    'DynamicsSensor': ['Odom', 'IMU'],
    'IMUSensor': ['', 'Bias']
    # TODO add Camera sensor and info topic
}

def _build_covariance(dim, cov=None, sigma=None):
    """
    Returns flattened NxN covariance list.

    Inputs (mutually exclusive):
    - cov: scalar, length-N, or NxN
    - sigma: scalar or length-N (converted to covariance)

    Default:
    - If both None → uses PERFECT_COV
    """

    if cov is not None and sigma is not None:
        raise ValueError("Cannot specify both covariance and sigma.")

    # --- sigma → covariance ---
    if sigma is not None:
        s = np.array(sigma, dtype=float)

        if s.ndim == 0:
            cov = float(s)**2
        elif s.shape == (dim,):
            cov = s**2
        else:
            raise ValueError(f"Sigma must be scalar or length-{dim}.")

    # --- default ---
    if cov is None:
        return (np.eye(dim) * float(PERFECT_COV)).flatten().tolist()

    c = np.array(cov, dtype=float)

    # scalar → σ² I
    if c.ndim == 0:
        return (np.eye(dim) * float(c)).flatten().tolist()

    # diagonal
    if c.shape == (dim,):
        return np.diag(c).flatten().tolist()

    # full matrix
    if c.shape == (dim, dim):
        return c.flatten().tolist()

    raise ValueError(f"Covariance must be scalar, length-{dim}, or {dim}x{dim}.")

def _get_partial_six_covariance(cov=None, sigma=None):
    """
    Returns flattened 6x6 covariance.

    Only fills top-left 3x3.
    """

    # Build 3x3 first
    cov3 = np.array(_build_covariance(3, cov=cov, sigma=sigma)).reshape(3, 3)

    # Embed into 6x6
    cov6 = np.zeros((6, 6))
    cov6[:3, :3] = cov3

    return cov6.flatten().tolist()


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

        if "socket" in sensor_dict and sensor_dict['socket'] != "":
            self.socket = sensor_dict['socket']
        else:
            self.socket = "base_link"
        
        self.socket = self.agent_name + "/" + self.socket

        self.publisher = None


    @abstractmethod
    def encode(self, sensor_data):
        pass

class IMUEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = Imu

        config = self.config or {}
        if self.type == 'IMUSensorBias':
            accel_cov_key = 'AccelBiasCov'
            accel_sigma_key = 'AccelBiasSigma'
            ang_cov_key = 'AngVelBiasCov'
            ang_sigma_key = 'AngVelBiasSigma'
        elif self.type == 'IMUSensor':
            accel_cov_key = 'AccelCov'
            accel_sigma_key = 'AccelSigma'
            ang_cov_key = 'AngVelCov'
            ang_sigma_key = 'AngVelSigma'
        else:
            raise ValueError(f"Unknown IMU sensor type: {self.type}")

        self.accel_cov = _build_covariance(
            dim=3,
            cov=config.get(accel_cov_key),
            sigma=config.get(accel_sigma_key),
        )

        self.ang_cov = _build_covariance(
            dim=3,
            cov=config.get(ang_cov_key),
            sigma=config.get(ang_sigma_key),
        )  
    
    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = self.socket
        msg.orientation_covariance[0] = UNKNOWN_COV

        if self.type == 'IMUSensorBias':
            accel_row = 2
            ang_row = 3
        elif self.type == 'IMUSensor':
            accel_row = 0
            ang_row = 1

        # Assign acceleration
        msg.linear_acceleration.x = float(sensor_data[accel_row, 0])
        msg.linear_acceleration.y = float(sensor_data[accel_row, 1])
        msg.linear_acceleration.z = float(sensor_data[accel_row, 2])

        # Assign angular velocity
        msg.angular_velocity.x = float(sensor_data[ang_row, 0])
        msg.angular_velocity.y = float(sensor_data[ang_row, 1])
        msg.angular_velocity.z = float(sensor_data[ang_row, 2])

        
        msg.linear_acceleration_covariance = self.accel_cov
        msg.angular_velocity_covariance = self.ang_cov

        return msg

class DVLEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)
        
        self.message_type = TwistWithCovarianceStamped

        config = self.config or {}

        cov = config.get('VelCov')
        sigma = config.get('VelSigma')

        #TODO: Holoocean Sensor sets covariance on each beam velocity lenght 4
        # FOR NOW we can only handle the case where it is a scalar
        if cov is not None:
            # Check cov is scalar
            if not isinstance(cov, (int, float)):
                raise ValueError("Velocity covariance must be a scalar.")
            
        if sigma is not None:
            # Check sigma is scalar
            if not isinstance(sigma, (int, float)):
                raise ValueError("Velocity sigma must be a scalar.")

        self.cov = _get_partial_six_covariance(
            cov=config.get('VelCov'),
            sigma=config.get('VelSigma'),
        )

    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = self.socket
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
        # TODO Range Covariance
        # TODO with update DVL sensor in HoloOcean

    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = self.socket

        msg.range[0] = float(sensor_data[3])
        msg.range[1] = float(sensor_data[4])
        msg.range[2] = float(sensor_data[5])
        msg.range[3] = float(sensor_data[6])

        return msg

class DepthEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = Odometry
        self.cov = [0.0] * 36
        self.cov[14] = PERFECT_COV  # Z position covariance

        if self.config is not None:
            if 'Cov' in self.config:
                self.cov[14] = float(self.config['Cov'])
            if 'Sigma' in self.config:
                self.cov[14] = float(self.config['Sigma']) ** 2
            

    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = 'holoocean_map'
        msg.child_frame_id = self.socket
        msg.pose.pose.position.z = float(sensor_data[0])
        msg.pose.covariance = self.cov
        return msg

class PoseSensorEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = PoseWithCovarianceStamped

    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = self.socket

        # Rotation
        rot_matrix = sensor_data[:3, :3]
        quat = Rotation.from_matrix(rot_matrix).as_quat()

        # Position
        msg.pose.pose.position.x = float(sensor_data[0, 3])
        msg.pose.pose.position.y = float(sensor_data[1, 3])
        msg.pose.pose.position.z = float(sensor_data[2, 3])

        # Orientation
        msg.pose.pose.orientation.x = float(quat[0])
        msg.pose.pose.orientation.y = float(quat[1])
        msg.pose.pose.orientation.z = float(quat[2])
        msg.pose.pose.orientation.w = float(quat[3])

        return msg

class LocationEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = PoseWithCovarianceStamped

        config = self.config or {}
        self.cov = _get_partial_six_covariance(cov=config.get('Cov'), sigma=config.get('Sigma'))

    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = self.socket
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
        rpy_msg.header.frame_id = self.socket
        rpy_msg.vector.x = float(sensor_data[0])
        rpy_msg.vector.y = float(sensor_data[1])
        rpy_msg.vector.z = float(sensor_data[2])
        return rpy_msg

class VelocityEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)
        
        self.message_type = TwistWithCovarianceStamped
        self.cov = _get_partial_six_covariance(cov=None, sigma=None)


    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = self.socket + "_world"
        #Frame id is global frame

        # Assign velocity
        msg.twist.twist.linear.x = float(sensor_data[0])
        msg.twist.twist.linear.y = float(sensor_data[1])
        msg.twist.twist.linear.z = float(sensor_data[2])
        msg.twist.covariance = self.cov

        return msg

class DynamicsEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)
        
        self.message_type = Odometry
        self.cov = _build_covariance(6, cov=None, sigma=None)

    def encode(self, sensor_data):
        msg = self.message_type()
        # TODO would need to check if UseCOM flag is set.
        msg.header.frame_id = 'holoocean_map'
        msg.child_frame_id = self.socket + "_world"
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

        msg.pose.covariance = self.cov
        msg.twist.covariance = self.cov

        return msg

class DynamicsIMUEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)
        
        self.message_type = Imu

        # Define arbitrary IMU covariance matrices
        self.cov = _build_covariance(3, cov=None, sigma=None)  

    def encode(self, sensor_data):
        msg = self.message_type()
        # TODO would need to check if UseCOM flag is set.
        msg.header.frame_id = self.socket + "_world"

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

        msg.orientation_covariance = self.cov
        msg.angular_velocity_covariance = self.cov
        msg.linear_acceleration_covariance = self.cov

        return msg

class GPSEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)
        
        self.message_type = Odometry
        config = self.config or {}

        self.cov = _get_partial_six_covariance(cov=config.get('Cov'), sigma=config.get('Sigma'))

    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = 'holoocean_map' # TODO get this from a param
        msg.child_frame_id = self.socket
        msg.pose.pose.position.x = float(sensor_data[0])
        msg.pose.pose.position.y = float(sensor_data[1])
        msg.pose.pose.position.z = float(sensor_data[2])
        msg.pose.covariance = self.cov
        return msg

class CommandEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = AgentCommand


    def encode(self, sensor_data):
        msg = self.message_type()
        msg.command = sensor_data.tolist()

        return msg
    
class ImageEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = Image
    
    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = self.socket

        # Remove the alpha channel (convert RGBA -> RGB)
        num_channels = 3  
        sensor_data = sensor_data[:, :, :num_channels]  # Keep only the first 3 channels

        # Ensure correct height and width
        msg.height = sensor_data.shape[0]  # Rows
        msg.width = sensor_data.shape[1]   # Columns

        # Step calculation
        msg.step = msg.width * num_channels  
        msg.encoding = "bgr8"
        msg.is_bigendian = 0

        # Convert to bytes
        msg.data = sensor_data.tobytes()

        # Debugging: Check expected vs actual size
        expected_size = msg.height * msg.step
        actual_size = len(msg.data)
        if expected_size != actual_size:
            print(f"ERROR: Expected data size {expected_size}, but got {actual_size}")

        return msg

class MagneticFieldEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = MagneticField
        config = self.config or {}
        self.cov = _build_covariance(3, cov=config.get('Cov'), sigma=config.get('Sigma'))

    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = self.socket
        # Assign magnetic field values
        msg.magnetic_field.x = float(sensor_data[0])
        msg.magnetic_field.y = float(sensor_data[1])
        msg.magnetic_field.z = float(sensor_data[2])
        msg.magnetic_field_covariance = self.cov
        return msg

class LaserScanEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = LaserScan
        
        count = 1
        range_max = 10.0

        if self.config is not None:
            if "LaserMaxDistance" in self.config:
                range_max = float(self.config["LaserMaxDistance"])
            if "LaserCount" in self.config:
                count = int(self.config["LaserCount"])

        self.msg_template = self.message_type()

        self.msg_template.header.frame_id = self.socket
        self.msg_template.angle_min = 0.0 # 0 degrees
        self.msg_template.angle_max = 6.28319   # 360 degrees
        self.msg_template.angle_increment = 6.28319 / count

        self.msg_template.range_min = 0.0
        self.msg_template.range_max = range_max


    def encode(self, sensor_data):
        msg = self.message_type()
        # Copy template fields
        msg.header.frame_id = self.msg_template.header.frame_id
        msg.angle_min = self.msg_template.angle_min
        msg.angle_max = self.msg_template.angle_max
        msg.angle_increment = self.msg_template.angle_increment
        msg.range_min = self.msg_template.range_min
        msg.range_max = self.msg_template.range_max

        msg.ranges = sensor_data.tolist()

        return msg

# Define other encoders similarly...


encoders = {
    'IMUSensor': IMUEncoder,
    'IMUSensorBias': IMUEncoder,
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
    'RGBCamera': ImageEncoder,
    'ViewportCapture': ImageEncoder,
    'MagnetometerSensor': MagneticFieldEncoder,
    'CameraSensor': ImageEncoder,
    'RangeFinderSensor': LaserScanEncoder,
    'PoseSensor': PoseSensorEncoder,
    # Add other sensor type encoders here...
}