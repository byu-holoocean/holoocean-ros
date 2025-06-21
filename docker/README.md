# Holoocean ROS Docker Containers

This repository provides two Docker container options for working with Holoocean and Holoocean ROS:

## 1. Development Container

- **Purpose:** For development and compilation.
- **Features:**
    - Holoocean and Holoocean ROS code are volume-mounted.
    - Includes build tools for compiling and running code.
    - Ideal for active development and testing.
- **Availability:**  
    - Pre-built image available on Docker Hub.
    - No need to build locally—simply pull and run.

## 2. Runtime Container

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

- **Display Issues:** If GUI apps fail to open, ensure `xhost +local:docker` has been run and your DISPLAY variable is set correctly.Make sure your X11 server is running and Docker is allowed access.
- **SSH errors cloning HoloOcean:** Confirm your GitHub SSH key is configured and you have access to the repo. You must sign the Epic Games EULA before proceeding.
- **Docker not found:** Ensure Docker is installed and running.