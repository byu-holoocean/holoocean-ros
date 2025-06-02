# Use the specified base image
FROM adamrehn/ue4-runtime:22.04-vulkan-x11

# Switch to root user to install dependencies
USER root
ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y --no-install-recommends \
    build-essential \
    curl \
    libglib2.0-dev \
    software-properties-common \
    python3-dev \
    python3-pip \
    sudo \
    libsm6 \
    libxrender-dev \
    libxext6 \
    libxrandr2 \
    x11-xserver-utils \
    && rm -rf /var/lib/apt/lists/*

# Add user to sudoer
RUN usermod -aG sudo $NAME
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
# TODO do these need to be run with UE4 user?
# Install Python packages (Needs to be before ros humble installation due to dependencies)
RUN pip3 install setuptools wheel tox posix_ipc numpy scipy matplotlib

# Setup ROS HUMBLE
RUN apt update && apt install -y locales
# TODO do I need the export LANG?
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && export LANG=en_US.UTF-8
# Add the ROS 2 repository
RUN add-apt-repository universe && apt update
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && apt install -y \
    ros-humble-desktop \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Source ROS setup script
# TODO: change this to the entrypoint? is this useful
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN source /opt/ros/humble/setup.bash

# Switch back to the ue4 user
USER ue4
WORKDIR /home/ue4
RUN mkdir -p .local/lib
RUN mkdir -p .local/share
RUN mkdir -p ros2_ws/src

# Copy the holoocean directory from tmp folder

# Copy the ros_packages in the tmp directory to the ros2_ws/src/ 


# Default command
CMD ["/bin/bash"]

