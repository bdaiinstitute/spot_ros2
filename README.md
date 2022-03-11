# Spot ROS2 

<img src="spot.jpeg" width="350">

## Prerequisites
    - Tested for ubuntu 20.04
    - ROS 2 foxy        
    - bosdyn-api

## Install
    pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core
    cd path/to/ros2/ws
    git clone git@git.fh-aachen.de:mk9955e/spot_ros2.git src/
    colcon build

## Launch
### Model
    ros2 launch spot_description description.launch.py
    rviz2       #new terminal

### SpotDriver
    ros2 launch spot_driver spot_driver.launch.py