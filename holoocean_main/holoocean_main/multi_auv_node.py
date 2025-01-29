import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import holoocean
import numpy as np

class MultiAUVNode(Node):
    def __init__(self):
        super().__init__('multi_auv_node')
        self.auv0_publisher_ = self.create_publisher(Float32MultiArray, 'auv0_sensors', 10)
        self.auv1_publisher_ = self.create_publisher(Float32MultiArray, 'auv1_sensors', 10)
        self.auv3_publisher_ = self.create_publisher(Float32MultiArray, 'auv3_sensors', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Timer to update both robots

        # HoloOcean Configuration
        self.cfg = {
            "name": "multi_auv_sim",
            "world": "SimpleUnderwater",
            "package_name": "Ocean",
            "main_agent": "auv0",
            "agents": [
                {
                    "agent_name": "auv0",
                    "agent_type": "TorpedoAUV",
                    "sensors": [
                        {"sensor_type": "IMUSensor"}
                    ],
                    "control_scheme": 0,
                    "location": [0, 0, -5]
                },
                {
                    "agent_name": "auv1",
                    "agent_type": "HoveringAUV",
                    "sensors": [
                        {"sensor_type": "DVLSensor"}
                    ],
                    "control_scheme": 0,
                    "location": [0, 2, -5]
                },
                {
                    "agent_name": "auv3",
                    "agent_type": "TorpedoAUV",
                    "sensors": [
                        {"sensor_type": "IMUSensor"}
                    ],
                    "control_scheme": 0,
                    "location": [0, -1, -5]
                }
            ],
        }

        # Initialize HoloOcean Environment
        self.env = holoocean.make(scenario_cfg=self.cfg)
        self.env.reset()

        self.get_logger().info("Multi-AUV Simulation Initialized")

    def timer_callback(self):
        try:
            # Act on AUV0
            self.env.act('auv0', np.array([0, 0, 0, 0, 75]))  # Example action
            # Act on AUV1
            self.env.act('auv1', np.array([0, 0, 0, 0, 20, 20, 20, 20]))  # Example action
            # Act on AUV3
            self.env.act('auv3', np.array([0, 0, 0, 0, 75]))  # Example action

            # Tick the simulation
            state = self.env.tick()

            # Extract and publish IMU data for AUV0
            imu_data = state["auv0"]["IMUSensor"].flatten()
            # self.get_logger().info(f"Raw IMU Data (AUV0): {imu_data}")
            imu_msg = Float32MultiArray()
            imu_msg.data = [float(value) if np.isfinite(value) else 0.0 for value in imu_data] 
            self.auv0_publisher_.publish(imu_msg)
            
            # Extract and publish DVL data for AUV1
            dvl_data = state["auv1"]["DVLSensor"].flatten()
            # self.get_logger().info(f"Raw DVL Data (AUV1): {dvl_data}")
            dvl_msg = Float32MultiArray()
            dvl_msg.data = [float(value) if np.isfinite(value) else 0.0 for value in dvl_data]  
            self.auv1_publisher_.publish(dvl_msg)
            
            # Extract and publish IMU data for AUV3
            imu_data_3 = state["auv3"]["IMUSensor"].flatten()
            # self.get_logger().info(f"Raw IMU Data (AUV3): {imu_data_3}")
            imu_msg_3 = Float32MultiArray()
            imu_msg_3.data = [float(value) if np.isfinite(value) else 0.0 for value in imu_data_3]  
            self.auv3_publisher_.publish(imu_msg_3)

            # Log the sensor data
            self.get_logger().info(f"AUV0 IMU: {imu_data}")
            self.get_logger().info(f"AUV1 DVL: {dvl_data}")
            self.get_logger().info(f"AUV3 IMU: {imu_data_3}")

        except Exception as e:
            self.get_logger().error(f"Error during simulation step: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MultiAUVNode()
    try:
        rclpy.spin(node)
    finally:
        node.env.close()  # Ensure proper cleanup of HoloOcean environment
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
