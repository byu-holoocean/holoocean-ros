# NOTICE: This Docker image can not be shared publicly due to licensing restrictions.
# Any docker image with the Worlds installed can only be shared internally 
# by someone who has accepted the Epic Games EULA.
# Stage 1: Build ros packages and pip install holoocean
FROM frostlab/holoocean:2.0-dev-ubuntu22.04-cuda12.4-ros AS builder

USER root
# Copy the client folder from the repository. 
# This assumes the build context is set to root of the holoocean-ros package
COPY ./docker/runtime/tmp/HoloOcean/client /tmp/holoocean_client
RUN python3 -m venv /opt/venv && \
    /opt/venv/bin/pip install --upgrade pip && \
    /opt/venv/bin/pip install /tmp/holoocean_client

COPY holoocean_examples ros2_ws/src/
COPY holoocean_interfaces ros2_ws/src/
COPY holoocean_main ros2_ws/src/

WORKDIR /home/ue4/ros2_ws
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Stage 2: Minimal runtime image
FROM nvidia/cuda:12.4.0-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV PATH="/opt/venv/bin:$PATH"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"

RUN apt-get update && apt-get install -y \
    python3 \
    libgl1-mesa-glx libglu1-mesa \
    libxrandr2 libxinerama1 libxcursor1 libxi6 \
    libsm6 libxext6 libxrender1 \
    libvulkan1 \
    locales \
    software-properties-common \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user 'ue4' with UID 1000
RUN useradd -m -u 1000 ue4

# Copy the virtual environment from the builder
COPY --from=builder /opt/venv /opt/venv

# Permissions fix for holoocean package content
RUN chown -R ue4:ue4 /opt/venv || true



# ROS2 Installation
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

# Add ROS 2 Humble repository
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-ros-base \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

USER ue4
# Source ROS setup for all users via bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ue4/.bashrc

COPY --from=builder /home/ue4/ros2_ws /home/ue4/ros2_ws
RUN echo "source /home/ue4/ros2_ws/install/setup.bash" >> /home/ue4/.bashrc

WORKDIR /home/ue4

# Install the worlds
# RUN /opt/venv/bin/python3 -c 'import holoocean; holoocean.install("TestWorlds")'
RUN /opt/venv/bin/python3 -c 'import holoocean; holoocean.install("Ocean")'

CMD ["/bin/bash"]