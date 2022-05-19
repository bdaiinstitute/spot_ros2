# Spot ROS2 Driver
This is a ROS 2 implemantion of the [ROS1 driver](https://github.com/clearpathrobotics/spot_ros) from Clearpath.
## Issues
All ROS services are ported, but only the services: claim, power_on, stand and sit were tested.
The ros actions have not yet been ported and the `cmd_vel` topic also were not yet tested.

<img src="spot.jpeg" width="350">

## Prerequisites
    - Tested for ubuntu 20.04
    - ROS 2 foxy

## Install
    pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core
    sudo apt install ros-foxy-joint-state-publisher-gui
    cd path/to/ros2/ws
    git clone https://github.com/MASKOR/Spot-ROS2.git src/
    colcon build --symlink-install

## Launch
The spot login data hostname, username and password must be specified in the `config/spot_login.yaml` in the spot_driver package.

### Model
    ros2 launch spot_description description.launch.py

### SpotDriver
    ros2 launch spot_driver spot_driver.launch.py