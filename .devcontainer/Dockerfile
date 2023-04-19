FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# install packages
RUN apt-get update \
    && apt-get install -q -y --no-install-recommends \
    curl \
    && rm -rf /var/lib/apt/lists/*

ARG ROS_DISTRO=humble
ENV ROS_DISTRO $ROS_DISTRO
ARG INSTALL_PACKAGE=base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN DEBIAN_FRONTEND=noninteractive apt-get update -q && \
    apt-get update -q && \
    apt-get install -yq --no-install-recommends \
    python3-pip \
    python-is-python3 \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-pytest-cov \
    python3-rosdep \
    python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# I added this
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Spot packages
RUN apt-get update -q && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-depth-image-proc \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-interactive-markers && \
    rm -rf /var/lib/apt/lists/*

# Install bosdyn_msgs package
RUN curl -sL https://github.com/bdaiinstitute/bosdyn_msgs/releases/download/v0.0.1-humble/ros-humble-bosdyn-msgs_0.0.1-0jammy_amd64.deb --output /tmp/ros-humble-bosdyn-msgs_0.0.1-0jammy_amd64.deb --silent \
  && dpkg -i /tmp/ros-humble-bosdyn-msgs_0.0.1-0jammy_amd64.deb \
  && rm /tmp/ros-humble-bosdyn-msgs_0.0.1-0jammy_amd64.deb

# ROS doesn't recognize the docker shells as terminals so force colored output
ENV RCUTILS_COLORIZED_OUTPUT=1
