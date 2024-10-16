Setup ros2 holoocean workspace in a container:

For ubuntu 22.04 follow the instructions here:
https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_local

Install cuda 12.5 and nvidia driver (my computer 555)

Follow these instructions for the container toolkit:

https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/1.16.0/install-guide.html

Install docker: Docker version 20.10.17, build 100c701


HOLOOCEAN:

Sign the unreal agreements

Clone the source code from: https://github.com/byu-holoocean/HoloOcean

Install the ros2 packages into the ros2 workspace from 
https://github.com/byu-holoocean/holoocean-ros/tree/main

Change the file paths for the holoocean repository and the ros2 workspace in the docker compose file

Run the following commands:

xhost +	 						#Give the container access to the screen
docker exec -it holoocean bash 	#Enter into the container
pip install holoocean/client 	#Install the holoocean 

You can check to make sure GPU access is available inside the container by running:
	nvidia-smi

To enter the ros2_ws and build the workspace inside the container:
	source /opt/ros/humble/setup.bash
	cd ~/ros2_ws 
	# rm -rf build/ install/ log #YOU MAY NEED TO REMOVE THESE FOLDERS IF BUILT OUTSIDE THE # CONTAINER
	colcon build  
	source install/setup.bash

Then to launch the torpedo node, state estimation, and command node you can run
	ros2 launch holoocean_main holoocean_torpedo_launch.py


#DOCKER setup:


Make sure you have
- container toolkit
- nvidia driver
- run xhost +