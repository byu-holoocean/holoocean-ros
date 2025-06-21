# Development Docker Environment

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
   git clone git@github.com:byu-holoocean/HoloOcean.git
   ```

3. **Install ROS 2 Packages**  
   Install the ROS 2 packages for HoloOcean into your ROS 2 workspace:
   ```
   git clone git@github.com:byu-holoocean/holoocean-ros.git
   ```

4. **Adjust File Paths**  
    Before starting the container, update the file paths in the `docker-compose` file to point to the cloned HoloOcean repository (specifically the `client` folder) and the holoocean-ros package.

5. **Run Setup Commands**  
   Enter the following commands to configure the container and access the HoloOcean environment:

   ```bash
   xhost +                             # Grant container access to the screen
   docker compose up -d                # Create a container and setup based on the holoocean ros image
   docker exec -it holoocean bash      # Enter the container
   ```

6. **Verify GPU Access**  
   Check GPU access within the container by running:
   ```bash
   nvidia-smi
   ```


## Notes

**Volume Mounts**  
    Both the HoloOcean repository and ROS packages are mounted as volumes into the container. This allows you to edit code on your host machine and have changes reflected inside the container immediately.

**Pre-built Image**  
    The Docker image used here is available online and does **not** need to be built locally. Simply pull and run the image as described in the usage instructions. Docker will automatically try and pull from docker hub when you run `docker compose up -d` and the image is not on the device.

## Usage

1. Clone the HoloOcean repository to your device.
2. Update the volume path in `docker-compose.yaml` to point to your local `holoocean/client` path.
3. Run `docker compose build` if you want to build the container
4. Run `docker compose up -d` if you want to start and detach from the container. 