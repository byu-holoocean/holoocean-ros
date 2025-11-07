from abc import ABC, abstractmethod
from sensor_msgs.msg import Imu, Image, MagneticField, LaserScan, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from holoocean_interfaces.msg import DVLSensorRange, ControlCommand
import numpy as np
from struct import pack, unpack

# TODO make a not about how the Dynamics Sensor IMU is not in local frame
multi_publisher_sensors = {
    'DVLSensor': ['Velocity', 'Range'],
    'DynamicsSensor': ['Odom', 'IMU']
    # TODO add Camera sensor and info topic
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

        if "socket" in sensor_dict and sensor_dict['socket'] != "":
            self.socket = sensor_dict['socket']
        else:
            self.socket = "base_link"

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
        msg.header.frame_id = self.socket
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

        self.message_type = PoseWithCovarianceStamped
        self.cov = [0.0] * 36

        if self.config is not None:
            if 'Cov' in self.config:
                self.cov[14] = float(self.config['Cov'])

    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = self.socket
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


    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = self.socket
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
        msg.header.frame_id = 'holoocean_global_frame'
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
        msg.header.frame_id = 'holoocean_global_frame'

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
        msg.header.frame_id = self.socket
        msg.pose.pose.position.x = float(sensor_data[0])
        msg.pose.pose.position.y = float(sensor_data[1])
        msg.pose.pose.position.z = float(sensor_data[2])
        msg.pose.covariance = self.cov
        return msg

class CommandEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = ControlCommand


    def encode(self, sensor_data):
        msg = self.message_type()
        msg.cs = sensor_data.tolist()

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

    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = self.socket
        # Assign magnetic field values
        msg.magnetic_field.x = float(sensor_data[0])
        msg.magnetic_field.y = float(sensor_data[1])
        msg.magnetic_field.z = float(sensor_data[2])
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

class LidarEncoder(SensorPublisher):
    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = PointCloud2


        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='t', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=20, datatype=PointField.UINT8, count=1),
            PointField(name='obj_tag', offset=21, datatype=PointField.UINT8, count=1),
            PointField(name='rgb', offset=22, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Define structured dtype for point cloud
        self.sem_pt_dtype = np.dtype({
            'names': ['x', 'y', 'z', 'intensity', 't', 'ring', 'obj_tag', 'rgb'],
            'formats': [np.float32, np.float32, np.float32, np.float32, 
                       np.float32, np.uint8, np.uint8, np.float32],
            'offsets': [0, 4, 8, 12, 16, 20, 21, 22]   
        })
        
        self.point_step = 26
        self.scans = 0
        self.full_lidar_scan = None

    
    def get_color_from_id(self, id):
        id_colors = {
            0: [0,0,0],   	    #       // None         =    0u,
            1: [79, 82, 77],    #		// Asphalt		 =    1u,
            2: [212, 191, 125], #		// Bench		 =	  2u,
            3: [209, 208, 203], #		// BikeRack		 =	  3u,
            4: [31, 240, 205],  #		// Building =   4u,
            5: [240, 220, 43],  #		// Bus =  5u,
            6: [177, 247, 124], #    	// Bush	 =	  6u,
            7: [224, 109, 237], #		// Car = 7u,
            8: [161, 247, 233], #		// Ceiling = 8u,
            9: [65, 13, 255],   #		// Chair = 9u,
            10:[255, 169, 10],  #		// Cone = 10u,
            11:[140, 86, 0],    #		// Crate = 11u,
            12:[242, 65, 224],  #		// Desk = 12u,
            13:[0, 138, 14],    #		// Dumpster = 13u,
            14:[255, 0, 0],     #		// FireHydrant = 14u,
            15:[2, 125, 104],   #		// Floor = 15u,
            16:[0, 184, 92],    #		// GarbageCan = 16u,
            17:[91, 235, 52],   # 		// Grass = 17u,
            18:[143, 129, 6],   #		// Pallet = 18u,
            19:[222, 164, 177], #		// ParkingGate = 19u,
            20:[195, 0, 255],   #		// PatioUmbrella = 20u,
            21:[242, 97, 130],  #		// Railing = 21u,
            22:[124, 6, 138],   #		// SemiTruck = 22u,
            23:[232, 232, 232], #		// Sidewalk = 23u,
            24:[222, 218, 245], #		// SpeedLimitSign = 24u,
            25:[250, 37, 62],   #		// StopSign = 25u,
            26:[255, 149, 10],  #		// StreetLamps = 26u,
            27:[186, 17, 169],  #		// Table = 27u,
            28:[69, 99, 46],    #		// Tree = 28u,
            29:[0, 0, 0],       #		// Unlabeled = 29u,
            30:[126, 60, 250],  #		// Wall = 30u,
            31:[107, 156, 137], # 		// Trash = 31u
            32:[255, 255, 255], #       // Any          =  0xFF    
        }
        rgb = id_colors[int(id)]
        rgb_int = (int(rgb[0]) << 16) | (int(rgb[1]) << 8) | int(rgb[2])
        rgb_float = unpack('f', pack('I', rgb_int))[0]  # Convert RGB uint32 to float32
        return rgb_float

    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = self.socket
        
        # sensor_data :[x, y, z, angle, obj_id, obj_instance]
        
        if self.full_lidar_scan is None and len(sensor_data) > 0:
            # print("LiDAR data len:",len(sensor_data))
            self.full_lidar_scan = sensor_data
        elif len(sensor_data) > 0:
            # print("LiDAR data len:",len(sensor_data))
            self.full_lidar_scan = np.vstack((self.full_lidar_scan,sensor_data))
        # elif self.full_lidar_scan is None:
    
        self.scans +=1    

        if self.scans == 30:
            # print("Updating prev scan")
            # print("Number of Accumulated Points:",self.full_lidar_scan.shape[0])
            points = self.full_lidar_scan
            self.scans = 0
            self.full_lidar_scan = None
        
        
            # shape: (N, 7)
            # print("Lidar points shape:", points.shape)
            
            # Create structured array for point cloud data
            # pc_data = np.zeros((points.shape[0], 1), dtype=self.sem_pt_dtype)
            pc_data = np.zeros(points.shape[0], dtype=self.sem_pt_dtype)
            
            # Populate point cloud data
            pc_data['x'] = points[:, 0]
            pc_data['y'] = points[:, 1]
            pc_data['z'] = points[:, 2]
            pc_data['intensity'] = points[:, 3]
            pc_data['t'] = 0.0  # relative time in nanoseconds
            pc_data['ring'] = points[:, 4].astype(np.uint8)
            # TODO WHY IS THIS 6 before
            pc_data['obj_tag'] = points[:, 5].astype(np.uint8)
            
            # Compute RGB colors from object IDs
            # TODO WHY IS THIS 6 before
            obj_ids = points[:, 5].astype(int)
            pc_data['rgb'] = np.array([self.get_color_from_id(obj_id) for obj_id in obj_ids])
            
            # Set message parameters
            # TODO check which fields can change with configuration
            msg.height = 1 
            msg.width = pc_data.shape[0] 
            msg.is_dense = True
            msg.is_bigendian = False
            msg.point_step = self.point_step
            msg.row_step = pc_data.shape[0] * self.point_step
            msg.fields = self.fields
            # msg.data = np.frombuffer(pc_data.tobytes(), dtype=np.uint8)
            msg.data = pc_data.tobytes()

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
    'RGBCamera': ImageEncoder,
    'MagnetometerSensor': MagneticFieldEncoder,
    'CameraSensor': ImageEncoder,
    'RangeFinderSensor': LaserScanEncoder,
    # Add other sensor type encoders here...
    'RaycastSemanticLidar': LidarEncoder,
    'RaycastLidar': LidarEncoder,
}