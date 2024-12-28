#!/bin/bash

# Function to handle `docker exec` commands
docker_exec() {
    docker exec -it holoocean bash -c "$1"
}

docker_python(){
    docker exec -it holoocean python3 -c "$1"
}

# Prompt to build the container or not
read -p "Do you want to build the container? (y/n): " build_container
if [[ "$build_container" == "y" || "$build_container" == "Y" ]]; then
    docker compose up -d --build
else
    docker compose up -d
fi

# Allow access to the X server
xhost +

# Install the HoloOcean client
docker_exec "pip install ~/holoocean/client"

# Prompt for world installation options
echo "Which worlds do you want to install?"
echo "1) Ocean"
echo "2) Testworlds"
echo "3) Both"
echo "*)Any other key to skip"
read -p "Enter your choice (1/2/3): " install_choice

case "$install_choice" in
    1) docker_python "import holoocean; holoocean.install('Ocean')";;
    2) docker_python "import holoocean; holoocean.install('TestWorlds')";;
    3) 
        docker_python "import holoocean; holoocean.install('Ocean')"
        docker_python "import holoocean; holoocean.install('TestWorlds')"
        ;;
    *)
        echo "Skipping world installation."
        ;;
esac

# Build the ROS workspace
docker_exec "source /opt/ros/humble/setup.bash && cd ~/ros2_ws && colcon build"

# Verify NVIDIA GPU compatibility inside the container
echo "Checking GPU compatibility inside the container..."
docker_exec 'if command -v nvidia-smi &> /dev/null; then
    echo "Running nvidia-smi to verify GPU support...";
    nvidia_smi_output=$(nvidia-smi);
    if echo "$nvidia_smi_output" | grep -q "CUDA"; then
        echo "NVIDIA GPU is enabled and working correctly.";
    else
        echo "Warning: NVIDIA GPU might not be enabled correctly.";
    fi;
else
    echo "nvidia-smi command not found. Ensure NVIDIA drivers are installed in the container.";
fi'
