# Use the specified base image
FROM frostlab/holoocean:2.0-dev-ubuntu22.04-cuda12.4

# Switch to root user to install dependencies
USER root
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt update && apt install -y --no-install-recommends \
    locales \
    software-properties-common \
    curl && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

# # Install Python packages (before ROS to avoid conflicts)
# RUN pip3 install --no-cache-dir \
#     setuptools \
#     wheel \
#     tox \
#     posix_ipc \
#     numpy \
#     scipy \
#     matplotlib


# Add ROS 2 Humble repository
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-dev-tools && \
    apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Source ROS setup for all users via bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ue4/.bashrc

# RUN apt update && apt install -y --no-install-recommends \
#     sudo && \
#     apt-get clean \
#     && rm -rf /var/lib/apt/lists/*

# Add user 'ue4' to sudoers with no password
RUN usermod -aG sudo ue4 && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Switch back to ue4 user and set up workspace
USER ue4
WORKDIR /home/ue4

# Create the ros2_ws folder owned by ue4
RUN mkdir -p /home/ue4/ros2_ws/src

# Add sourcing of ROS workspace to bashrc
RUN echo 'source /home/ue4/ros2_ws/install/setup.bash' >> /home/ue4/.bashrc

# Default command
CMD ["/bin/bash"]
