# ROS 2 Wrapper

After installing HoloOcean, you can clone this ROS package into your ROS workspace:
[https://github.com/byu-holoocean/holoocean-ros](https://github.com/byu-holoocean/holoocean-ros)

### Important Notes

- **Only single-agent scenarios are currently supported.**
- Running HoloOcean and ROS 2 in a virtual environment (e.g., Conda) may cause dependency issues.
- The simulation speed will max out, potentially causing timing errors.

This HoloOcean ROS workspace is compatible with ROS 2 Humble. Follow the ROS 2 installation tutorials here:
[ROS 2 Humble Installation](https://docs.ros.org/en/humble/Tutorials)

### Running the Example Controller

After building the package in your ROS workspace, you can run the example controller with the following commands:

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

ros2 launch holoocean_main torpedo_launch.py
```

Refer to the ROS 2 documentation to build nodes that can control the HoloOcean environment.

### `Torpedo` Node Overview

- **Subscribes** to controller commands
- **Publishes** sensor data returned in the HoloOcean state
- **Creates a timer** to tick the environment

HoloOcean publishers use the `holoocean` namespace. The topic name will follow the scenario name for each sensor. If no name is provided, it defaults to the sensor type.

### HoloOcean Interface Package

- Contains the messages and services used for publishing sensor data.

### `command_example` Node

- Publishes random Heading-Speed-Depth (HSD) commands at specific intervals.
- Can publish a preprogrammed sequence of HSD commands.

## Publishing Your Own Sensor

To integrate a new sensor in both Python and C++ with the HoloOcean package:

1. **Create a Message File**  
   Add a `.msg` file (e.g., `SensorData.msg`) in `holoocean_interfaces`.

2. **Add to CMakeLists**  
   Update the `CMakeLists.txt` file to include the new message (e.g., `"msg/SensorData.msg"`).

3. **Update Data Converter File**  
   - Add an `elif` statement in the `convert_to_msg` function.
   - Register a new key in `sensor_keys`.
   - Import the message object (e.g., `from holoocean_interfaces.msg import SensorData`).
   - Add a function to encode the data into a ROS message.

Then, rebuild your ROS 2 workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Recording Your Data

To record sensor data or commands with `ros2bag`, run a command like this from your `ros2_ws` folder:

```bash
source install/setup.bash
ros2 bag record /holoocean/desiredHSD /holoocean/RotationSensor /holoocean/LocationSensor -o /path/to/save_data
```

For more information on recording and playing back ROS 2 bag data, see the [ROS 2 Bag Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html).

---

### About HoloOcean

HoloOcean is a high-fidelity simulator developed by the Field Robotic Systems Lab (FRostLab) at Brigham Young University. Built upon Unreal Engine (by Epic Games) and Holodeck (developed by the BYU PCCL Lab), HoloOcean facilitates easy simulation of marine robotics and autonomy with a wide variety of sensors, agents, and features.

This repository provides the ROS 2 packages necessary to connect HoloOcean to the ROS 2 network.

### Prerequisites

- ROS 2 workspace (tested on ROS 2 Humble Hawksbill): [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
- HoloOcean installation:
  - Source Code: [https://github.com/byu-holoocean/HoloOcean](https://github.com/byu-holoocean/HoloOcean)
  - Documentation: [HoloOcean Documentation](https://byu-holoocean.github.io/holoocean-docs/)

