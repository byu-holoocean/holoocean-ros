## HoloOcean ROS2 Runtime Docker Container

This container is not packaged and provided on DockerHub due to the EULA licensing with Unreal Engine.


## Usage

1. **Unreal Engine Agreements**  
   Ensure you have signed the Unreal Engine agreements.

1. **Clone the repository** and navigate to the `docker/runtime` folder:

   ```bash
   git clone git@github.com:byu-holoocean/holoocean-ros.git
   cd holoocean-ros/docker/runtime
   ```

2. **Build the container:**

   ```bash
   ./build_container.sh
   ```

   - The script will:
     - Check for Docker installation
     - Prompt for EULA confirmation
     - Guide you through X11 display access setup
     - Clone the HoloOcean repository into a tmp folder (requires SSH access)
     - Builds the Docker image
     - Optionally start the container

3. **Display Access (for GUI):**
   - If prompted, allow Docker to access your X11 display:
     ```bash
     xhost +local:docker
     ```
   - **Note:** This command must be re-run after every reboot.


## **build_container.sh**

#### **Command-line Options**

- `-b `: Specify a git branch for HoloOcean (default: `develop`)


## **Usage Tips**

- **Rebuilding:** If you change the Dockerfile or dependencies, re-run the script to rebuild the image. If you plan to rebuild consistently the dev image is better suited for frequent changes.

## **Security Notice**

> **WARNING:**  
> This image contains proprietary software and is intended for internal use only due to EULA with Epic Games.
> **Do NOT share a built docker image outside your organization.**

