# Development Docker Environment

Instructions for setting up a ROS 2 HoloOcean workspace in a development container environment on Ubuntu 22.04.

## Usage

1. **Unreal Engine Agreements**  
   Ensure you have signed the Unreal Engine agreements.

2. **Clone the Source Code**  
   Clone the HoloOcean source code from GitHub:
   ```
   git clone git@github.com:byu-holoocean/HoloOcean.git
   ```

3. **Clone ROS 2 Packages**  
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
   
   # OPTIONAL: 
   # docker compose build  # Build container locally instead of pulling from docker hub
   
   docker compose up -d                # Create a container and setup based on the holoocean ros image
   
   docker exec -it holoocean bash      # Enter the container
   ```


## Notes

**Volume Mounts**  
    Both the HoloOcean repository and ROS packages are mounted as volumes into the container. This allows you to edit code on your host machine and have changes reflected inside the container immediately.

**Pre-built Image**  
    The Docker image used here is available online and does **not** need to be built locally. Simply pull and run the image as described in the usage instructions. Docker will automatically try and pull from docker hub when you run `docker compose up -d` and the image is not on the device.

