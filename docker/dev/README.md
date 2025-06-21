# Development Docker Environment

This folder contains resources for running a development Docker container for working with HoloOcean and ROS2.

## Key Points

1. **Update HoloOcean Path**  
    Before starting the container, update the path to your local HoloOcean repository (specifically the `client` folder) in the Docker configuration files. This ensures the container has access to your local code.

2. **Volume Mounts**  
    Both the HoloOcean repository and ROS packages are mounted as volumes into the container. This allows you to edit code on your host machine and have changes reflected inside the container immediately.

3. **Pre-built Image**  
    The Docker image used here is available online and does **not** need to be built locally. Simply pull and run the image as described in the usage instructions. Docker will automatically try and pull from docker hub when you run `docker compose up -d` and the image is not on the device.

## Usage

1. Clone the HoloOcean repository to your device.
2. Update the volume path in `docker-compose.yaml` to point to your local `holoocean/client` path.
3. Run `docker compose build` if you want to build the container
4. Run `docker compose up -d` if you want to start and detach from the container. 