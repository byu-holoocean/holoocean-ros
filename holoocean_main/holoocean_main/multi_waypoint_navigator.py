import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import holoocean
import numpy as np
import math


from loguru import logger

class MultiWaypointNavigator(Node):
    @logger.catch
    def __init__(self):
        super().__init__('multi_waypoint_navigator')

        self.path_sv = []  # To store GPS coordinates for sv
        self.path_auv0 = []  # To store GPS coordinates for auv0
        self.path_auv1 = []  # To store GPS coordinates for auv1

        self.publisher_sv = self.create_publisher(Float32MultiArray, 'sv_waypoint', 10)
        self.publisher_auv0 = self.create_publisher(Float32MultiArray, 'auv0_waypoint', 10)
        self.publisher_auv1 = self.create_publisher(Float32MultiArray, 'auv1_waypoint', 10)

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.config = {
            "name": "MultiWaypointSim",
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
                    "control_scheme": 1,
                    "location": [0, 0, 2],
                    "rotation": [0, 0, 0]
                },
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
                }
            ],
        }

        # Waypoints for sv, auv0, and auv1
        self.locations_sv = np.array([[25, 25], [-25, 25], [-25, -25], [25, -25]])
        self.locations_auv0 = np.array([[15, 15], [-15, 15], [-15, -15], [15, -15]])
        self.locations_auv1 = np.array([[10, 10], [-10, 10], [-10, -10], [10, -10]])

        self.idx_sv = 0
        self.idx_auv0 = 0
        self.idx_auv1 = 0

        self.env = holoocean.make(scenario_cfg=self.config)
        self.env.reset()
        self.init_simulation()

    @logger.catch
    def init_simulation(self):
        # Draw waypoints for each vehicle in the HoloOcean environment
        for location in self.locations_sv:
            self.env.draw_point([location[0], location[1], 2], lifetime=0)
        for location in self.locations_auv0:
            self.env.draw_point([location[0], location[1], -5],color= [255, 242, 94], lifetime=0)
        for location in self.locations_auv1:
            self.env.draw_point([location[0], location[1], -5],color= [30, 34, 165], lifetime=0)
        self.get_logger().info("Starting simulation.")

    @logger.catch
    def xyz_to_gps(self, x, y, z, ref_lat, ref_lon, ref_alt, scale=1.0):
        earth_radius = 6378137.0
        delta_lat = (y * scale) / earth_radius
        lat = ref_lat + math.degrees(delta_lat)
        delta_lon = (x * scale) / (earth_radius * math.cos(math.radians(ref_lat)))
        lon = ref_lon + math.degrees(delta_lon)
        alt = ref_alt + (z * scale)
        return lat, lon, alt

    @logger.catch
    def timer_callback(self):
        try:
            state = self.env.tick()

            # Surface Vehicle (SV) Navigation
            sv_position = state["sv"]["GPSSensor"][0:3]
            waypoint_sv = self.locations_sv[self.idx_sv]
            relative_waypoint_sv = waypoint_sv - sv_position[:2]
            distance_sv = np.linalg.norm(relative_waypoint_sv)
            rudder_angle = np.arctan2(relative_waypoint_sv[1], relative_waypoint_sv[0])
            throttle = min(distance_sv * 0.1, 1.0)
            sv_action = np.array([rudder_angle, throttle], dtype=np.float32)
            if distance_sv < 2.0:
                self.idx_sv = (self.idx_sv + 1) % len(self.locations_sv)
            self.env.act("sv", sv_action)

            # AUV0 Navigation
            auv0_imu = np.array(state["auv0"]["IMUSensor"])
            auv0_position = auv0_imu[0, :2]
            waypoint_auv0 = self.locations_auv0[self.idx_auv0]
            relative_waypoint_auv0 = waypoint_auv0 - auv0_position
            distance_auv0 = np.linalg.norm(relative_waypoint_auv0)
            yaw_rate_auv0 = np.arctan2(relative_waypoint_auv0[1], relative_waypoint_auv0[0])
            speed_auv0 = min(distance_auv0 * 0.5, 75)
            auv0_action = np.array([speed_auv0, 0.0, 0.0, 0.0, 0.0, yaw_rate_auv0], dtype=np.float32)
            if distance_auv0 < 2.0:
                self.idx_auv0 = (self.idx_auv0 + 1) % len(self.locations_auv0)
            self.env.act("auv0", auv0_action)

            # AUV1 Navigation
            auv1_dvl = np.array(state["auv1"]["DVLSensor"])
            auv1_position = auv1_dvl[:2]
            waypoint_auv1 = self.locations_auv1[self.idx_auv1]
            relative_waypoint_auv1 = waypoint_auv1 - auv1_position
            distance_auv1 = np.linalg.norm(relative_waypoint_auv1)
            yaw_rate_auv1 = np.arctan2(relative_waypoint_auv1[1], relative_waypoint_auv1[0])
            speed_auv1 = min(distance_auv1 * 0.3, 50)
            auv1_action = np.array([speed_auv1, 0.0, 0.0, 0.0, 0.0, yaw_rate_auv1], dtype=np.float32)
            if distance_auv1 < 2.0:
                self.idx_auv1 = (self.idx_auv1 + 1) % len(self.locations_auv1)
            self.env.act("auv1", auv1_action)

            # Debugging Logs
            self.get_logger().info(f"SV Position: {sv_position}, Distance: {distance_sv}, Action: {sv_action}")
            self.get_logger().info(f"AUV0 Position: {auv0_position}, Distance: {distance_auv0}, Action: {auv0_action}")
            self.get_logger().info(f"AUV1 Position: {auv1_position}, Distance: {distance_auv1}, Action: {auv1_action}")

        except Exception as e:
            self.get_logger().error(f"Error in simulation: {e}")


    # @logger.catch
    # def timer_callback(self):
    #     try:
    #         # Step simulation and get the state
    #         state = self.env.tick()

    #         # Surface Vehicle Position and Waypoint Navigation
    #         sv_position = state["sv"]["GPSSensor"][0:3]
    #         waypoint_sv = self.locations_sv[self.idx_sv]
    #         direction_sv = np.array(waypoint_sv) - sv_position[:2]
    #         distance_sv = np.linalg.norm(direction_sv)
    #         normalized_direction_sv = direction_sv / distance_sv if distance_sv > 0 else np.zeros(2)
    #         sv_action = np.array([*normalized_direction_sv, 5], dtype=np.float32)
    #         self.get_logger().info(f"SV Position: {sv_position}, Distance: {distance_sv}, Action: {sv_action}")

    #         # Torpedo AUV (AUV0)
    #         auv0_position = state["auv0"]["IMUSensor"][0]  # Extract the first row if multidimensional
    #         waypoint_auv0 = self.locations_auv0[self.idx_auv0]
    #         direction_auv0 = np.array(waypoint_auv0) - auv0_position[:2]
    #         distance_auv0 = np.linalg.norm(direction_auv0)
    #         normalized_direction_auv0 = direction_auv0 / distance_auv0 if distance_auv0 > 0 else np.zeros(2)
    #         speed_auv0 = min(distance_auv0 * 5, 75)
    #         auv0_action = np.array([*normalized_direction_auv0, 0, 0, speed_auv0], dtype=np.float32)
    #         self.get_logger().info(f"AUV0 Position: {auv0_position}, Distance: {distance_auv0}, Speed: {speed_auv0}")

    #         # Hovering AUV (AUV1)
    #         auv1_position = state["auv1"]["DVLSensor"][:, 0:3].mean(axis=0)  # Average across rows if multidimensional
    #         waypoint_auv1 = self.locations_auv1[self.idx_auv1]
    #         direction_auv1 = np.array(waypoint_auv1) - auv1_position[:2]
    #         distance_auv1 = np.linalg.norm(direction_auv1)
    #         normalized_direction_auv1 = direction_auv1 / distance_auv1 if distance_auv1 > 0 else np.zeros(2)
    #         speed_auv1 = min(distance_auv1 * 2 + 10, 50)
    #         auv1_action = np.array([*normalized_direction_auv1, 0, 0, *([speed_auv1] * 4)], dtype=np.float32)
    #         self.get_logger().info(f"AUV1 Position: {auv1_position}, Distance: {distance_auv1}, Speed: {speed_auv1}")

    #         # Act in the environment
    #         self.env.act("sv", sv_action)
    #         self.env.act("auv0", auv0_action)
    #         self.env.act("auv1", auv1_action)

    #     except Exception as e:
    #         self.get_logger().error(f"Error during simulation step: {e}")


    @logger.catch
    def save_path(self):
        with open("path_coordinates.txt", "w") as f:
            f.write("SV Path:\n")
            for lat, lon in self.path_sv:
                f.write(f"{lat},{lon}\n")
            f.write("\nAUV0 Path:\n")
            for lat, lon in self.path_auv0:
                f.write(f"{lat},{lon}\n")
            f.write("\nAUV1 Path:\n")
            for lat, lon in self.path_auv1:
                f.write(f"{lat},{lon}\n")

def main(args=None):
    rclpy.init(args=args)
    node = MultiWaypointNavigator()
    try:
        rclpy.spin(node)
    finally:
        node.env.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
