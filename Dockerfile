# Use official Ubuntu 22.04 base image
FROM ubuntu:22.04

#RMW ZENOH experimental flag
ARG EXPERIMENTAL_ZENOH_RMW=FALSE

# Set noninteractive mode for APT
ENV DEBIAN_FRONTEND=noninteractive

# Env setup
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

# Install system dependencies
RUN apt-get update && apt-get install -y locales curl git \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update

# Install dependencies
RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
    wget software-properties-common lsb-release gnupg2 \
    python3-colcon-common-extensions python3-pip python3-pybind11 \
    python3-pytest-cov python3-tk python3-rosdep python3-colcon-mixin \
    python3-rosinstall-generator python3-vcstool libpython3-dev \
    ros-humble-ros-base ros-dev-tools \
    #check if Zenoh should be installed
    $(if [ "$EXPERIMENTAL_ZENOH_RMW" = "TRUE" ]; then echo "ros-humble-rmw-zenoh-cpp"; fi) \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /ros_ws/src

# Initialize rosdep
RUN rosdep init && rosdep update

# Clone driver code
RUN git clone https://github.com/bdaiinstitute/spot_ros2.git .
RUN git submodule update --init --recursive

# Run install script and pass in the architecture
RUN ARCH=$(dpkg --print-architecture) && echo "Building Driver with Architecture: $ARCH"
RUN /ros_ws/src/install_spot_ros2.sh $ARCH

# Build packages with Colcon
WORKDIR /ros_ws/
RUN /bin/bash -c "source /opt/ros/humble/setup.sh && colcon build --symlink-install"
