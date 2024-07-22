# Use the specified base image
FROM adamrehn/ue4-runtime:22.04-cudagl11-noaudio

# Switch to root user to install dependencies
USER root
ENV DEBIAN_FRONTEND=noninteractive

# Update and install necessary dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    curl \
    libglib2.0-dev \
    software-properties-common \
    libsm6 \
    libxrender-dev \
    libxext6 \
    libxrandr2 \
    x11-xserver-utils \
    python3-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install setuptools wheel tox posix_ipc numpy==1.26 scipy

# Setup ROS Humble
# Add the ROS 2 repository
RUN apt-get update && apt-get install -y software-properties-common \
    && add-apt-repository universe \
    && add-apt-repository restricted \
    && add-apt-repository multiverse \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && apt-get update \
    && apt-get install -y ros-humble-ros-base \
    && apt-get clean

# Source ROS setup script
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Switch back to the ue4 user
USER ue4

# Default command
CMD ["/bin/bash"]

#Docker compose to share project directory and also test worlds

# docker build -t frostlab/holoocean:ros



