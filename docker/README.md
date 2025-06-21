# Holoocean ROS Docker Containers
This folder contains all the resources needed to build and run a Docker container for the HoloOcean project, integrating Unreal Engine 5 (UE5), ROS 2 Humble, and the HoloOcean client. The setup is designed for GPU-accelerated simulation with X11 display forwarding and supports custom ROS package integration.


## Usage

See the README files in the dev and runtime folders for usage instructions for the specific containers.


## Prerequisites


1. **CUDA and NVIDIA Drivers**  
   For Ubuntu 22.04, follow the instructions [here](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_local) to download and install CUDA 12.4 and the NVIDIA driver (for example, driver version 555).

2. **NVIDIA Container Toolkit**  
   Install the NVIDIA container toolkit by following the instructions [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/1.16.0/install-guide.html).
   (Specifically the installation of the container toolkit and configuring docker for use with the nvidia container toolkit)

4. **Docker**  
   Install Docker (tested with Docker version 20.10.17, build 100c701).

## Container Options

This repository provides two Docker container options for working with Holoocean and Holoocean ROS:

### 1. Development Container

- **Purpose:** For development and compilation.
- **Features:**
    - Holoocean and Holoocean ROS code are volume-mounted.
    - Includes build tools for compiling and running code.
    - Ideal for active development and testing.
- **Availability:**  
    - Pre-built image available on Docker Hub.
    - No need to build locally—simply pull and run.


### 2. Runtime Container

- **Purpose:** For running pre-built Holoocean applications.
- **Features:**
    - Minimal environment without build tools.
    - Holoocean and Holoocean ROS code are pre-installed.
    - Suitable for deployment and production use.
- **Availability:**  
    - **Not distributed online** due to Epic Games EULA restrictions.
    - Image can only be shared internally.
    - A build script is provided to help you build and start the container locally.


## **Troubleshooting**

- **Display Issues:** If GUI apps fail to open, check:
    - `xhost +local:docker` has been run 
    - DISPLAY enviornment variable is set.
    - Make sure your X11 server is running and Docker is allowed access.

- **SSH errors cloning HoloOcean:** Confirm your GitHub SSH key is configured and you have access to the repo. You must sign the Epic Games EULA before proceeding.

- **Verify GPU Access**  
   Check GPU access within the container by running:
   ```bash
   nvidia-smi
   ```

- **Docker not found:** Ensure Docker is installed and running.