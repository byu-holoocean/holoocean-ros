# holoocean_examples

Example nodes for interacting with the HoloOcean ROS 2 interface.

## Examples

| Launch file | Description |
|---|---|
| `joy_launch.py` | Single-controller joystick teleoperation with optional camera HUD |
| `multi_joy_launch.py` | Same as above but supports a wired and wireless controller simultaneously |
| `waypoint_launch.py` | Waypoint following for a surface vessel |
| `command_launch.py` | Direct depth/heading/speed commanding via the Fossen autopilot |

---

## Joystick Example

### Prerequisites

Install the `joy_linux` ROS 2 package if not already present:

```bash
sudo apt install ros-$ROS_DISTRO-joy-linux
```

Connect your controller. Verify the device shows up:

```bash
ls /dev/input/js*
```
You can setup udev rules to map specific controllers to specific paths. 

The config files default to `js0`. Adjust `dev` under `joy_node` (or `joy_node_wired`in multi agent) in the config YAML if your device path differs.

### Running

```bash
ros2 launch holoocean_examples joy_launch.py
```

For multiple agents and multiple controllers simultaneously:

```bash
ros2 launch holoocean_examples multi_joy_launch.py
```

### How agents are loaded

The node reads `scenario_path` from the config (same value used by `holoocean_node`) and parses the scenario JSON to get the agent list. You do **not** need to list agents in the YAML — only the button assignments.

`agents.buttons` is a parallel array where index `i` maps to the i-th agent in the scenario file. For example, if the scenario defines `[auv0, auv1, auv2]` in order:

```yaml
agents.buttons: [0, 1, 3]   # auv0=A, auv1=B, auv2=Y
agents.default: 'auv0'
```

`agents.default` sets which agent is active at startup.

---

### Finding your button and axis mapping

The easiest way to find the indices for your specific controller is to echo the joy topic while pressing buttons:

```bash
# Run with only one controller connected
ros2 run joy_linux joy_linux_node

# In another terminal run:
ros2 topic echo /joy
```

The `buttons` array shows `1` for any button currently held. The `axes` array shows the current value of each analog input (sticks and triggers).

The [ROS joy wiki page](https://wiki.ros.org/joy) also has mapping tables for common controllers.

### Default mapping (Xbox controller via joy_linux)

The values below match the defaults in `joy_config.yaml` (wired controller) and `multi_joy_config.yaml`.

#### Buttons

| Index | Xbox button | Function |
|---|---|---|
| 0 | A | Select agent 0 |
| 1 | B | Select agent 1 (if present) |
| 2 | X | Select agent 2 (if present) |
| 3 | Y | Select agent 3 (if present) |
| 6 | Back / Select | Disarm |
| 7 | Start | Arm |
| 8 | Guide (Xbox logo) | Reset simulation |

> The wireless Xbox config in `joy_config.yaml` uses buttons 6/7/8. Some controllers or drivers report these at different indices — use `ros2 topic echo /joy` to confirm.

#### Axes

| Index | Xbox input | Function |
|---|---|---|
| 0 | Left stick X | Lateral (strafe) |
| 1 | Left stick Y | Forward / Pitch (torpedo) |
| 2 | LT | Tilt camera down |
| 3 | Right stick X | Yaw (BlueROV) / Yaw (torpedo) |
| 4 | Right stick Y | Vertical (BlueROV) / Speed (torpedo) |
| 5 | RT | Tilt camera up |
| 6 | D-pad X | Roll trim (BlueROV) |
| 7 | D-pad Y | Pitch trim (BlueROV) |

> Trigger axes rest at `1.0` and reach `-1.0` when fully pressed. This is the standard `joy_linux` convention.

### Per-agent controls

#### BlueROV2 / HoveringAUV

| Input | Effect |
|---|---|
| Left stick Y | Forward thrust |
| Left stick X | Lateral (strafe) thrust |
| Right stick Y | Vertical thrust |
| Right stick X | Yaw |
| D-pad up/down | Pitch trim (accumulates, resets on disarm) |
| D-pad left/right | Roll trim (accumulates, resets on disarm) |
| RT / LT | Tilt camera up / down |

#### SurfaceVessel

| Input | Effect |
|---|---|
| Left stick Y | Left thruster |
| Right stick Y | Right thruster |
| RT / LT | Tilt camera up / down (if agent has a CameraSensor) |

#### TorpedoAUV / CougUV

| Input | Effect |
|---|---|
| Left stick Y | Pitch fins |
| Left stick X | Yaw fins |
| Right stick Y | Thruster speed |
| RT / LT | Tilt camera up / down (if agent has a CameraSensor) |

---

### Camera HUD

The `camera_hud` node subscribes to a `CameraSensor` image and the `DynamicsSensorOdom` odometry topic for a given agent, overlays a pilot HUD, and republishes to `<agent>/CameraHUD`.

View the output with:

```bash
ros2 run rqt_image_view rqt_image_view
```

Then select `/holoocean/<agent>/CameraHUD` (or whichever agent) from the topic dropdown.

The `agent_name` parameter controls which agent the HUD follows. In `joy_config.yaml` and `multi_joy_config.yaml` this is set under the `camera_hud` node section.

---

## Config file structure

```
config/
  joy_config.yaml         Single controller, single or multi-agent scenario
  multi_joy_config.yaml   Two controllers, multi-agent scenario
  waypoint_config.yaml    Waypoint follower parameters
  sv_waypoints.yaml       Waypoint list for the surface vessel example
```

All YAML files follow the ROS 2 node parameter convention.
The wildcard entry at the top of each file:

```yaml
/holoocean/**:
  ros__parameters:
    relative_path: true
    scenario_path: 'config/my_scenario.json'
```

applies `relative_path` and `scenario_path` to **all** nodes in the `holoocean` namespace, so you only need to set the scenario in one place.
