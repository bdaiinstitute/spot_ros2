<p align="center">
  <img src="spot.png" width="350">
  <h1 align="center">Spot ROS2 Driver</h1>
  <p align="center">
    <a href="https://github.com/MASKOR/spot_ros2/blob/main/LICENSE">
      <img src="https://img.shields.io/badge/License-MIT-yellow.svg" />
    </a>
    <a href="https://www.python.org/">
        <img src="https://img.shields.io/badge/built%20with-Python3-red.svg" />
    </a>
    <a href="https://github.com/jiuguangw/Agenoria/actions">
    <img src="https://github.com/jiuguangw/Agenoria/actions/workflows/test.yml/badge.svg">
    </a>
  </p>
</p>

# Overview
This is a ROS2 package for BostonDynamics' Spot. The package contains all necessary topics, services and actions to teleoperate or navigate Spot.
This package is derived of this [ROS1 package](https://github.com/heuristicus/spot_ros).

## Prerequisites
    - Tested for Ubuntu 20.04 + Foxy
    - Tested for Ubuntu 22.04 + Humble

## Install
    pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core
    sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-xacro
    wget -q -O /tmp/ros-humble-bosdyn-msgs_0.0.0-0jammy_amd64.deb https://github.com/bdaiinstitute/bosdyn_msgs/releases/download/v0.0.0-humble/ros-humble-bosdyn-msgs_0.0.0-0jammy_amd64.deb
    sudo dpkg -i /tmp/ros-humble-bosdyn-msgs_0.0.0-0jammy_amd64.deb
    rm /tmp/ros-humble-bosdyn-msgs_0.0.0-0jammy_amd64.deb
    wget -q -O /tmp/ros-humble-spot-msgs_0.0.0-0jammy_amd64.deb https://github.com/bdaiinstitute/spot_ros2/releases/download/spot_msgs-v0.0-0/ros-humble-spot-msgs_0.0.0-0jammy_amd64.deb
    sudo dpkg -i /tmp/ros-humble-spot-msgs_0.0.0-0jammy_amd64.deb
    rm /tmp/ros-humble-spot-msgs_0.0.0-0jammy_amd64.deb
    cd <path/to/ros2/ws>
    git clone https://github.com/MASKOR/Spot-ROS2.git src/
    colcon build --symlink-install

### Install depth image proc
Since `DepthCloud` is not yet ported for rviz2 , we can use [depth_image_proc](http://wiki.ros.org/depth_image_proc) to visualize the depth information from the cameras as `Pointcloud2`.

    sudo apt install ros-$ROS_DISTRO-depth-image-proc

## Launch
The spot login data hostname, username and password can either be specified as ROS parameters or as environment variables.  If using ROS parameters, see `spot_driver/config/spot_ros_example.yaml` for an example of what your file could look like.  If using environment variables, define `BOSDYN_CLIENT_USERNAME`, `BOSDYN_CLIENT_PASSWORD`, and `SPOT_IP`.

### Model
    ros2 launch spot_description description.launch.py

### SpotDriver
    ros2 launch spot_driver spot_driver.launch.py config_file:=<path to your ROS config file>

### Depth image to Pointcloud2
    ros2 launch spot_driver point_cloud_xyz.launch.py

### Example Node
The `command_spot_driver` node contains service and action clients. To send a trajectory goal execute:

    ros2 run spot_driver command_spot --ros-args -p command:=trajectory

### Multiple Robots
If you want to use multiple robots, use the `spot_driver_with_namespace` launch file:

    ros2 launch spot_driver_with_namespace.py spot_name:=<spot name> config_file:=<path to your ROS config file>

This will launch all nodes in the `spot_name` namespace and use `spot_name/` as the prefix for all frames.

## License

MIT license - parts of the code developed specifically for ROS2.
BSD3 license - parts of the code derived from the Clearpath Robotics ROS1 driver.

## Contributors

This project is a collaboration between the [Mobile Autonomous Systems & Cognitive Robotics Institute](https://maskor.fh-aachen.de/en/) (MASKOR) at [FH Aachen](https://www.fh-aachen.de/en/) and the [Boston Dynamics AI Institute](https://theaiinstitute.com/).

MASKOR contributors:

* Maximillian Kirsch
* Shubham Pawar
* Christoph Gollok
* Stefan Schiffer
* Alexander Ferrein

Boston Dynamics AI Institute contributors:

* Jenny Barry
* Daniel Gonzalez
* David Surovik
* Jiuguang Wang

[Linköping University](https://liu.se/en/organisation/liu/ida) contributors:

* Tommy Persson
