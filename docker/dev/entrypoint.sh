#!/bin/bash

# Change the owner of the holoocean client folder due to pip installation errors
sudo chown ue4:ue4 -R /home/ue4/holoocean_client
# Pip install holoocean python package
pip install /home/ue4/holoocean_client

# Source and build ros2_ws
source /opt/ros/humble/setup.bash
cd /home/ue4/ros2_ws
colcon build

# Keeps the container running
exec tail -f /dev/null