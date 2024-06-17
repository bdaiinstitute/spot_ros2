FROM osrf/ros:humble-desktop-full

# Install dependencies (taken from devcontainer dockerfile)
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

WORKDIR /repo

COPY . .

RUN sed -i 's/git@github.com:/https:\/\/github.com\//' .gitmodules
RUN git submodule update --init --recursive
