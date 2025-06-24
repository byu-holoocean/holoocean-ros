# HoloOcean ROS 2 Interface

This repository provides ROS 2 integration for [HoloOcean](https://github.com/byu-holoocean/HoloOcean), a high-fidelity marine robotics simulator. It enables publishing sensor data and receiving control commands via ROS 2 topics and messages.

## Prerequisites

- ROS 2 (tested on ROS 2 Humble Hawksbill): [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
- HoloOcean installation:
  - Source Code: [https://github.com/byu-holoocean/HoloOcean](https://github.com/byu-holoocean/HoloOcean)
  - Documentation: [HoloOcean Documentation](https://byu-holoocean.github.io/holoocean-docs/)

## Installation

After installing HoloOcean, clone this repository into your ROS 2 workspace:

```bash
cd ros2_ws/src
git clone https://github.com/byu-holoocean/holoocean-ros.git
cd ..
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## ⚠️ Notes

* **You must use the `develop` branch of HoloOcean** for ROS 2 compatibility.
* Avoid using virtual environments (e.g., Conda) for ROS 2 and HoloOcean; they may cause runtime or dependency issues.
* The simulation will run at maximum speed by default. This may result in timing mismatches with real-time ROS controllers.

## Launching the Simulation

To run the node:

```
ros2 launch holoocean_main holoocean_launch.py
```

## Node Overview

### `holoocean_node`

This is the main simulation interface node. It:

* **Loads a scenario configuration file** and launches the HoloOcean environment.
* **Subscribes** to:

  * `command/control` for raw control input
  * `command/agent` for actuator-level commands
  * `depth`, `heading`, and `speed` for individual autopilot inputs
* **Publishes**:

  * Sensor data from all active HoloOcean agents
  * A `/clock` topic for simulation time
* **Provides services**:

  * `reset`: Reset the simulation
  * `control_mode`: Change an agent’s control mode (e.g., manual or autopilot)
* **Ticks the environment** in a background thread for smooth simulation and timing integration.

Sensor topics follow this format:

```
/holoocean/<SensorName>
```

If no name is set for the sensor in the scenario, it defaults to the sensor type.

## ⚙️ Node Parameters

The following parameters can be set for `holoocean_node` using a launch file or YAML config:

| Parameter          | Type   | Default | Description                                                                        |
| ------------------ | ------ | ------- | ---------------------------------------------------------------------------------- |
| `publish_commands` | bool   | `true`  | Whether to publish control commands to the simulator.                              |
| `show_viewport`    | bool   | `true`  | Whether to show the Unreal Engine viewport window.                                 |
| `draw_arrow`       | bool   | `true`  | Whether to draw an arrow indicating vehicle heading in the sim.                    |
| `render_quality`   | int    | `1`     | Adjust render quality (0 = low, 1 = normal, 2 = high).                             |
| `relative_path`    | bool   | `true`  | Whether to resolve `scenario_path` relative to the package directory.              |
| `scenario_path`    | string | `""`    | Path to the scenario JSON file, relative or absolute depending on `relative_path`. |

---

## Docker 

See the docker folder for running holoocean with ros in a docker container.
Runtime and development containers available. 

---


## 📦 Recording Sensor Data

Use `ros2 bag` to record topics:

```bash
ros2 bag record /holoocean/desiredHSD /holoocean/RotationSensor /holoocean/LocationSensor -o ~/rosbags/holoocean_test
```

See the [ROS 2 Bag Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) for more info.

---

## 📚 Resources

* **HoloOcean Repository**
  [https://github.com/byu-holoocean/HoloOcean](https://github.com/byu-holoocean/HoloOcean)

* **HoloOcean Documentation**
  [https://byu-holoocean.github.io/holoocean-docs/](https://byu-holoocean.github.io/holoocean-docs/)

* **ROS 2 Documentation**
  [https://docs.ros.org/en/humble/index.html](https://docs.ros.org/en/humble/index.html)

---

## 🛠 Developed By

The [FRoStLab (Field Robotic Systems Lab)](https://frostlab.byu.edu/) at Brigham Young University. Built on top of Unreal Engine and Holodeck, HoloOcean provides a platform for autonomous marine robotics research using ROS 2.

### About HoloOcean

HoloOcean is a high-fidelity simulator developed by the Field Robotic Systems Lab (FRostLab) at Brigham Young University. Built upon Unreal Engine (by Epic Games) and Holodeck (developed by the BYU PCCL Lab), HoloOcean facilitates easy simulation of marine robotics and autonomy with a wide variety of sensors, agents, and features.

This repository provides the ROS 2 packages necessary to connect HoloOcean to the ROS 2 network.
