#!/bin/bash

# Remove previously installed build cache
sudo rm -rf /home/ue4/holoocean_client/src/holoocean.egg-info /home/ue4/holoocean_client/build

# Pip install holoocean python package
pip install /home/ue4/holoocean_client

source /opt/ros/humble/setup.bash
cd /home/ue4/ros2_ws
colcon build

# Keeps the container running
exec tail -f /dev/null