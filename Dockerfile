FROM osrf/ros:humble-desktop-full-jammy

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ARG ROS_DISTRO=humble
ENV ROS_DISTRO $ROS_DISTRO
ARG INSTALL_PACKAGE=base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN DEBIAN_FRONTEND=noninteractive apt-get update -q && \
    apt-get update -q && \
    apt-get install -yq --no-install-recommends \
    lcov \
    curl \
    python3-pip \
    python-is-python3 \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    libpython3-dev \
    python3-vcstool \
    ros-humble-tl-expected && \
    rm -rf /var/lib/apt/lists/*

# I added this
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Install bosdyn_msgs package
RUN curl -sL https://github.com/bdaiinstitute/bosdyn_msgs/releases/download/4.0.0-2/ros-humble-bosdyn_msgs_4.0.0-2-jammy_amd64.run --output /tmp/ros-humble-bosdyn_msgs_4.0.0-2-jammy_amd64.run --silent \
  && chmod +x /tmp/ros-humble-bosdyn_msgs_4.0.0-2-jammy_amd64.run \
  && ((yes || true) | /tmp/ros-humble-bosdyn_msgs_4.0.0-2-jammy_amd64.run) \
  && rm /tmp/ros-humble-bosdyn_msgs_4.0.0-2-jammy_amd64.run

# Install spot_cpp_sdk package
RUN curl -sL https://github.com/bdaiinstitute/spot-cpp-sdk/releases/download/4.0.0/spot-cpp-sdk_4.0.0_amd64.deb --output /tmp/spot-cpp-sdk_4.0.0_amd64.deb --silent \
  && dpkg -i /tmp/spot-cpp-sdk_4.0.0_amd64.deb \
  && rm /tmp/spot-cpp-sdk_4.0.0_amd64.deb

# Install bosdyn_msgs missing dependencies
RUN python -m pip install --no-cache-dir --upgrade pip==22.3.1 \
    && pip install --root-user-action=ignore --no-cache-dir --default-timeout=900 \
    numpy==1.24.1 \
    pytest-cov==4.1.0 \
    pytest-xdist==3.5.0 \
    bosdyn-api==4.0.0 \
    bosdyn-core==4.0.0 \
    bosdyn-client==4.0.0 \
    bosdyn-mission==4.0.0 \
    bosdyn-choreography-client==4.0.0 \
    pip cache purge

# Install spot_wrapper requirements
RUN --mount=type=bind,source=spot_wrapper,target=/tmp/context/spot_wrapper cd /tmp/context && \
    pip install --root-user-action=ignore --no-cache-dir --default-timeout=900 -r spot_wrapper/requirements.txt && \
    pip cache purge

# Install packages dependencies
RUN --mount=type=bind,source=.,target=/tmp/context \
    apt-get update -q && rosdep update && \
    rosdep install -y -i --from-paths /tmp/context --skip-keys "bosdyn bosdyn_msgs spot_wrapper" && \
    rm -rf /var/lib/apt/lists/*

# ROS doesn't recognize the docker shells as terminals so force colored output
ENV RCUTILS_COLORIZED_OUTPUT=1
