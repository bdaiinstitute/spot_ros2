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
This is a ROS 2 implementation of the [ROS1 driver](https://github.com/clearpathrobotics/spot_ros) from Clearpath Robotics.

All ROS services are ported, but only the services: claim, power_on, stand and sit were tested.

The `/cmd_vel` topic also works and you can send commands to the spot via e.g. rqt_publisher.

This project is still a WIP.

## Issues
The ros actions have not yet been ported. The RobotModel is not visible in rviz so far.
DepthCloud is not visible in rviz2 either, because DepthCloud has not been ported for rviz2 yet.
The `spot_viz` package is also missing.

## Prerequisites
    - Tested for Ubuntu 20.04 + Foxy
    - Tested for Ubuntu 22.04 + Humble

## Install
    pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core
    sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-xacro
    cd <path/to/ros2/ws>
    git clone https://github.com/MASKOR/Spot-ROS2.git src/
    colcon build --symlink-install

### Install depth image proc
Since `DepthCloud` is not yet ported for rviz2 , we can use [depth_image_proc](http://wiki.ros.org/depth_image_proc) to visualize the depth information from the cameras as `Pointcloud2`.

    sudo apt install ros-$ROS_DISTRO-depth-image-proc

## Launch
The spot login data hostname, username and password must be specified in the `config/spot_login.yaml` of the spot_driver package.

### Model
    ros2 launch spot_description description.launch.py

### SpotDriver
    ros2 launch spot_driver spot_driver.launch.py

### Depth image to Pointcloud2
    ros2 launch spot_driver point_cloud_xyz.launch.py

## License

MIT license - parts of the code developed specifically for ROS2.
BSD3 license - parts of the code derived from the Clearpath Robotics ROS1 driver.

## Contributors

This project is a collaboration between the Mobile Autonomous Systems & Cognitive Robotics Institute (MASKOR) at FH Aachen and the Boston Dynamics AI Institute.

MASKOR contributors:

* Maximillian Kirsch
* Simon Roder
* Alexander Ferrein

Boston Dynamics AI Institute contributors:

* Jenny Barry
* Daniel Gonzalez
* David Surovik
* Jiuguang Wang
