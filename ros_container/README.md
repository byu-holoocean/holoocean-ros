Setup ROS 2 HoloOcean Workspace in a Container
=============================================

Instructions for setting up a ROS 2 HoloOcean workspace in a container environment on Ubuntu 22.04.

Prerequisites
-------------

1. **CUDA and NVIDIA Drivers**  
   For Ubuntu 22.04, follow the instructions [here](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_local) to download and install CUDA 12.5 and the NVIDIA driver (for example, driver version 555).

2. **Container Toolkit**  
   Install the NVIDIA container toolkit by following the instructions [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/1.16.0/install-guide.html).
   (Specifically the installation of the container toolkit and configuring docker for use with the nvidia container toolkit)

4. **Docker**  
   Install Docker (tested with Docker version 20.10.17, build 100c701).

HoloOcean Setup
---------------

1. **Unreal Engine Agreements**  
   Ensure you have signed the Unreal Engine agreements.

2. **Clone the Source Code**  
   Clone the HoloOcean source code from GitHub:
   ```
   git clone https://github.com/byu-holoocean/HoloOcean
   ```

3. **Install ROS 2 Packages**  
   Install the ROS 2 packages for HoloOcean into your ROS 2 workspace:
   ```
   git clone https://github.com/byu-holoocean/holoocean-ros.git ~/ros2_ws/src
   ```

4. **Adjust File Paths**  
   Update the file paths in the `docker-compose` file to point to the cloned HoloOcean repository and the ROS 2 workspace.

5. **Run Setup Commands**  
   Enter the following commands to configure the container and access the HoloOcean environment:

   ```bash
   xhost +                             # Grant container access to the screen
   docker exec -it holoocean bash      # Enter the container
   pip install holoocean/client        # Install HoloOcean client
   ```

6. **Verify GPU Access**  
   Check GPU access within the container by running:
   ```bash
   nvidia-smi
   ```

Building the ROS 2 Workspace
----------------------------

To enter the `ros2_ws` workspace and build it within the container:

1. **Source ROS 2 Environment**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Enter ROS 2 Workspace**
   ```bash
   cd ~/ros2_ws
   # You may need to remove these folders if the workspace was built outside the container
   rm -rf build/ install/ log
   ```

3. **Build the Workspace**
   ```bash
   colcon build
   ```

4. **Source the Workspace**
   ```bash
   source install/setup.bash
   ```

Launching HoloOcean Nodes
-------------------------

To launch the torpedo node, state estimation, and command nodes:

```bash
ros2 launch holoocean_main holoocean_torpedo_launch.py
```

Docker Setup
------------

Make sure you have the following components configured:

- NVIDIA Container Toolkit
- NVIDIA driver
- Run `xhost +` to grant display access to the container.
