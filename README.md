# Spot ROS2 

<img src="spot.jpeg" width="350">

## Prerequisites
    - Tested for ubuntu 20.04
    - ROS 2 foxy        
    - bosdyn-api

## Install
    pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core
    sudo apt install ros-foxy-joint-state-publisher-gui
    cd path/to/ros2/ws
    git clone https://github.com/MASKOR/Spot-ROS2.git src/
    colcon build --symlink-install

## Launch
The spot login data hostname, username and password must be specified in the config/spot_login.yaml in the spot_driver package.
### Model
    ros2 launch spot_description description.launch.py

### SpotDriver
    ros2 launch spot_driver spot_driver.launch.py