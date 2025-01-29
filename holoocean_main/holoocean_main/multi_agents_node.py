import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import holoocean
import numpy as np

class MultiAgentNode(Node):
    def __init__(self):
        super().__init__('multi_agent_node')
        self.auv0_publisher_ = self.create_publisher(Float32MultiArray, 'auv0_sensors', 10)
        self.auv1_publisher_ = self.create_publisher(Float32MultiArray, 'auv1_sensors', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # HoloOcean Configuration
        self.cfg = {
            "name": "multi_agent_sim",
            "world": "SimpleUnderwater",
            "package_name": "Ocean",
            "main_agent": "sv",
            "agents": [
                {
                    "agent_name": "sv",
                    "agent_type": "SurfaceVessel",
                    "sensors": [
                        {"sensor_type": "GPSSensor"}
                    ],
                    "control_scheme": 1,  # PD Control Scheme
                    "location": [0, 0, 2]
                },
                {
                    "agent_name": "auv0",
                    "agent_type": "TorpedoAUV",
                    "sensors": [
                        {"sensor_type": "IMUSensor"}
                    ],
                    "control_scheme": 0,
                    # "location": [10, 10, -10]
                    "location": [0, 0, -5]
                },
                {
                    "agent_name": "auv1",
                    "agent_type": "HoveringAUV",
                    "sensors": [
                        {"sensor_type": "DVLSensor"}
                    ],
                    "control_scheme": 0,
                    # "location": [-10, -10, -5]
                    "location": [0, 2, -5]
                }
            ],
        }

        # Initialize HoloOcean Environment
        self.env = holoocean.make(scenario_cfg=self.cfg)
        self.env.reset()

        # Waypoints for SV
        self.sv_waypoints = np.array([
            [25, 25],
            [-25, 25],
            [-25, -25],
            [25, -25]
        ])
        
        self.auv0_waypoints = np.array([[10, 10, -5],
                                        [-10, 10, -5],
                                        [-10, -10, -5],
                                        [10, -10, -5]])
        
        self.auv1_waypoints = np.array([[15, 15, -7],
                                        [-15, 15, -7],
                                        [-15, -15, -7],
                                        [15, -15, -7]])
        
        self.sv_idx = 0
        self.auv0_idx = 0
        self.auv1_idx = 0

        # Draw waypoints
        for wp in self.sv_waypoints:
            self.env.draw_point([wp[0], wp[1], 0], lifetime=0)
            
        for wp0 in self.auv0_waypoints:
            self.env.draw_point([wp0[0], wp0[1], -5],color= [255,255,0], lifetime=0)
            
        for wp1 in self.auv0_waypoints:
            self.env.draw_point([wp1[0], wp1[1], -7],color= [176,224,230], lifetime=0)

        self.get_logger().info("Multi-Agent Simulation Initialized...")

    def timer_callback(self):
        try:
            state = self.env.tick()

            # SV Waypoint Navigation
            if "sv" in state and "GPSSensor" in state["sv"]:
                gps_data = state["sv"]["GPSSensor"]
                self.get_logger().info(f"SV GPS Data: {gps_data}")
                sv_position = gps_data[:2]  # Extract x, y position

                waypoint = self.sv_waypoints[self.sv_idx]
                distance = np.linalg.norm(sv_position - waypoint)

                self.get_logger().info(f"SV Distance to waypoint: {distance}")

                if distance < 1e-1:  # If close to waypoint, switch to the next
                    self.sv_idx = (self.sv_idx + 1) % len(self.sv_waypoints)
                    self.get_logger().info(f"SV moving to waypoint {self.sv_idx}")
                    waypoint = self.sv_waypoints[self.sv_idx]  # Update waypoint

                # Send waypoint as action to the SV
                action = np.array([waypoint[0], waypoint[1], 0.0])  # Include z-axis
                self.env.act("sv", action)

            # AUV0 Waypoint Navigation
            if "auv0" in state and "IMUSensor" in state["auv0"]:
                imu_data = state["auv0"]["IMUSensor"]
                accel = imu_data[0]
                angular_velocity = imu_data[1]
                self.get_logger().info(f"AUV0 Acceleration: {accel}, Angular Velocity: {angular_velocity}")

                auv0_position = self.auv0_waypoints[self.auv0_idx][:3]  # Extract x, y, z
                distance = np.linalg.norm(auv0_position - self.auv0_waypoints[self.auv0_idx])

                self.get_logger().info(f"AUV0 Distance to waypoint: {distance}")

                if distance < 1e-1:  # If close to waypoint, switch to the next
                    self.auv0_idx = (self.auv0_idx + 1) % len(self.auv0_waypoints)
                    self.get_logger().info(f"AUV0 moving to waypoint {self.auv0_idx}")

                action = np.array([auv0_position[0], auv0_position[1], auv0_position[2], 0, 75])
                self.env.act('auv0', action)

            # AUV1 Waypoint Navigation
            if "auv1" in state and "DVLSensor" in state["auv1"]:
                dvl_data = state["auv1"]["DVLSensor"]
                velocity = dvl_data[:3]  # Extract velocity components
                self.get_logger().info(f"AUV1 Velocity: {velocity}")

                auv1_position = self.auv1_waypoints[self.auv1_idx][:3]  # Extract x, y, z
                distance = np.linalg.norm(auv1_position - self.auv1_waypoints[self.auv1_idx])

                self.get_logger().info(f"AUV1 Distance to waypoint: {distance}")

                if distance < 1e-1:  # If close to waypoint, switch to the next
                    self.auv1_idx = (self.auv1_idx + 1) % len(self.auv1_waypoints)
                    self.get_logger().info(f"AUV1 moving to waypoint {self.auv1_idx}")

                action = np.array([auv1_position[0], auv1_position[1], auv1_position[2], 20, 20, 20, 20])
                self.env.act('auv1', action)

        except Exception as e:
            self.get_logger().error(f"Error during simulation step: {e}")




def main(args=None):
    rclpy.init(args=args)
    node = MultiAgentNode()
    try:
        rclpy.spin(node)
    finally:
        node.env.close()  # Cleanup HoloOcean environment
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
