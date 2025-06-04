# Use the specified base image
FROM adamrehn/ue4-runtime:22.04-vulkan-x11

# Switch to root user to install dependencies
USER root
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
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
    locales && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

# Install Python packages (before ROS to avoid conflicts)
RUN pip3 install --no-cache-dir \
    setuptools \
    wheel \
    tox \
    posix_ipc \
    numpy \
    scipy \
    matplotlib

# Add user 'ue4' to sudoers with no password
RUN usermod -aG sudo ue4 && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Add ROS 2 Humble repository
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Source ROS setup for all users via bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ue4/.bashrc

# Switch back to ue4 user and set up workspace
USER ue4
WORKDIR /home/ue4

# Create local user directories (for pip, cache, etc.)
RUN mkdir -p .local/lib .local/share ros2_ws/src

COPY tmp/HoloOcean /home/ue4/holoocean

# Change ownership of the files to the ue4 user
USER root
RUN chown -R ue4:ue4 /home/ue4/ros2_ws/src && \
    chown -R ue4:ue4 /home/ue4/holoocean
USER ue4

# TODO make the option to use the a Different branch of the worlds install
RUN pip3 install /home/ue4/holoocean/client && \ 
    python3 -c "import holoocean; holoocean.install('Ocean')" && \
    rm -r /home/ue4/holoocean

# Uncomment the lines below if you want to copy and build the ros2 packages into the container
# Will have to remove the volume from the docker compose
# COPY tmp/ros_packages /home/ue4/ros2_ws/src
# # Source ROS 2 and then run colcon build in a single shell
# WORKDIR /home/ue4/ros2_ws
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Add sourcing of ROS workspace to bashrc
RUN echo 'source /home/ue4/ros2_ws/install/setup.bash' >> /home/ue4/.bashrc

# Default command
CMD ["/bin/bash"]
