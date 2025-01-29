import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import holoocean
import numpy as np

import math

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('sv_waypoint_navigator')
        
        self.path = []  # To store GPS coordinates - for pfolium plotting
         
        self.publisher_ = self.create_publisher(Float32MultiArray, 'current_waypoint', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.config = {
            "name": "SurfaceNavigator",
            "world": "SimpleUnderwater",
            "package_name": "Ocean",
            "main_agent": "sv",
            "draw_arrow": True,
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
                }
            ],
        }

        self.locations = np.array([[25, 25], [-25, 25], [-25, -25], [25, -25]])
        self.idx = 0
        self.env = holoocean.make(scenario_cfg=self.config)
        self.state = None
        self.init_simulation()

    def init_simulation(self):
        # Draw waypoints in the HoloOcean environment
        for location in self.locations:
            self.env.draw_point([location[0], location[1], 0], lifetime=0)
        self.get_logger().info(f"Starting simulation. Going to waypoint : {self.idx}")
        

    def xyz_to_gps(self, x, y, z, ref_lat, ref_lon, ref_alt, scale=1.0):
        """
        Converts simulation (x, y, z) coordinates to GPS (lat, lon, alt).

        Parameters:
            x, y, z: Local simulation coordinates
            ref_lat, ref_lon, ref_alt: Reference GPS coordinates
            scale: Scale factor for converting simulation units to meters

        Returns:
            lat, lon, alt: GPS coordinates
        """
        earth_radius = 6378137.0
        delta_lat = (y * scale) / earth_radius
        lat = ref_lat + math.degrees(delta_lat)
        delta_lon = (x * scale) / (earth_radius * math.cos(math.radians(ref_lat)))
        lon = ref_lon + math.degrees(delta_lon)
        alt = ref_alt + (z * scale)
        return lat, lon, alt

    def draw_arrows(self, current_position, waypoint):
        """
        Draws arrows:
        - Horizontal arrow: current position to waypoint
        - Vertical arrow: current position to the surface
        """
        x_curr, y_curr, z_curr = current_position
        x_wpt, y_wpt = waypoint

        # Horizontal arrow to waypoint
        self.env.draw_arrow(
            start=[x_curr, y_curr, z_curr],
            end=[x_wpt, y_wpt, z_curr],  # Same depth
            color=[0, 255, 0],  # Green
            thickness=5,
            lifetime=0.1
        )

        # Vertical arrow to surface
        self.env.draw_arrow(
            start=[x_curr, y_curr, z_curr],
            end=[x_curr, y_curr, 0],  # Surface at z=0
            color=[255, 0, 0],  # Red
            thickness=2,
            lifetime=0.1
        )
        
    def timer_callback(self):
        try:
            # Get the current simulation state
            self.state = self.env.step(self.locations[self.idx])
            
            # Extract current position (x, y, z)            
            sim_position = self.state["GPSSensor"][0:3]
            x_curr, y_curr, z_curr = sim_position
            
            
            ref_lat, ref_lon, ref_alt = 40.7128, -74.0060, 0 # Reference Point
            lat, lon, alt = self.xyz_to_gps(
                x_curr, y_curr, z_curr,
                # sim_position[0], sim_position[1], sim_position[2],
                ref_lat, ref_lon, ref_alt, scale=10.0
            )
            self.path.append((lat, lon))  # Store path
            self.get_logger().info(f"GPS Coordinates: [{lat}, {lon}, {alt}]")
            
            # Get the current waypoint
            waypoint = self.locations[self.idx]
            # Draw a line from the current position to the waypoint
            self.draw_arrows(sim_position, waypoint)
        
            if np.linalg.norm(sim_position[:2] - waypoint[:2]) < 0.1:
                self.idx = (self.idx + 1) % len(self.locations)
                self.get_logger().info(f"Going to waypoint {self.idx}")
        except Exception as e:
            self.get_logger().error(f"Error in simulation step: {e}")
            
    def save_path(self):
        with open("path_coordinates.txt", "w") as f:
            for lat, lon in self.path:
                f.write(f"{lat},{lon}\n")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.env.close()  # Ensure proper cleanup of the HoloOcean environment
    node.save_path()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
