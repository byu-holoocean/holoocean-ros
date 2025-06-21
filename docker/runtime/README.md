## HoloOcean ROS2 Official Docker Container

This folder contains all the resources needed to build and run a Docker container for the HoloOcean project, integrating Unreal Engine 4 (UE4), ROS 2 Humble, and the HoloOcean client. The setup is designed for GPU-accelerated simulation with X11 display forwarding and supports custom ROS package integration.
This container is not packaged and provided on DockerHub due to the EULA licensing with Unreal Engine.

---

### **Contents**

- `Dockerfile` – Builds the main container image, layering UE4 runtime, system dependencies, Python packages, ROS 2 Humble, and the HoloOcean client.
- `docker-compose.yml` – Defines the container runtime configuration, including GPU access, display forwarding, and volume mounts for code and data sharing.
- `run_docker.sh` – Bash script to automate setup, build, and startup, including repository cloning, and EULA compliance checks.

---

## **Quick Start**

### **Prerequisites**

- Docker installed
- NVIDIA GPU drivers and [NVIDIA Docker runtime](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- X11 server running (for GUI support)
- SSH key configured for GitHub access to private repositories
- Epic Games EULA linked with github account

---

### **Setup Steps**

1. **Clone the repository** and navigate to the `docker` folder:

   ```bash
   git clone 
   cd /docker
   ```

2. **Run the setup script:**

   ```bash
   ```

   - The script will:
     - Check for Docker installation
     - Prompt for EULA confirmation
     - Guide you through X11 display access setup
     - Clone the HoloOcean repository (requires SSH access)
     - Optionally include ROS packages and examples
     - Build the Docker image
     - Optionally start the container

3. **Display Access (for GUI):**
   - If prompted, allow Docker to access your X11 display:
     ```bash
     xhost +local:docker
     ```
   - **Note:** This command must be re-run after every reboot.

---

## **Configuration Details**

### **Dockerfile Highlights**

- **Base Image:** `nvidia/cuda:12.4.0-runtime-ubuntu22.04` (Ubuntu 22.04 with Cuda support)
- **Python Packages:** Pre-installs scientific stack (`numpy`, `scipy`, `matplotlib`), `setuptools`, and HoloOcean client dependencies.
- **ROS 2 Humble:** Installs core ROS 2 packages and sets up environment sourcing.
- **User Management:** Ensures the `ue4` user has passwordless sudo for development flexibility.
- **Workspace Setup:** Copies HoloOcean client and (optionally) ROS packages into the container.

### **docker-compose.yml**

- **GPU Support:** Uses the `nvidia` runtime for hardware acceleration.
- **Display Forwarding:** Shares the host X11 socket for GUI applications.
- **Network:** Uses `host` mode for seamless ROS networking.
- **Privileged Mode:** Required for some device access and X11.

### **build_container.sh**

#### **Command-line Options**

- `-b `: Specify a git branch for HoloOcean (default: `develop`)


## **Usage Tips**

- **Rebuilding:** If you change the Dockerfile or dependencies, re-run the script to rebuild the image. If you plan to rebuild consistently the dev image is better suited for frequent changes.

## **Security Notice**

> **WARNING:**  
> This image contains proprietary software and is intended for internal use only due to EULA with Epic Games.
> **Do NOT share a built docker image outside your organization.**

