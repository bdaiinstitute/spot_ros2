# Spot ROS2 

<img src="spot.jpeg" width="350">

The spot_ros2 packages is currently not working.
This package is supposed to be an interface between ROS2 and the boston api for the spot. The [ROS1 driver](https://github.com/clearpathrobotics/spot_ros) from Clearpath is used as a template for the development.

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

## Integration Notes

The **main()** function of the *spot_ros2* is not anymore a method of the *SpotROS* class.
Also, the **shutdown()** function is not anymore registered and executed by a ROS handler, since ROS2 doesn't support it. Therefore, the **shutdown()** function is now registered and called system-wide via signal.

The while loop of the **main()** function which calls **updateTasks()** and publishs feedback and mobility params is now encapsulated in its own function called **step()**.
The **step()** function is now called by thread.
The reason is, normally at the end of the **main()** the created ROS node has to **spin()** to update the messages on the topics. The **spin()** function blocks the execution.
Since I didn't find where Clearpath spin inside the ROS1 driver, i decided to provide this while loop via a new function with threading.

# Known issues
![](https://fh-aachen.sciebo.de/s/VG7ZnE83ysFwF9h/download)

![](https://fh-aachen.sciebo.de/s/GAiqn2oLmLznwc6/download)

![](https://fh-aachen.sciebo.de/s/1mIsHaYS3wnzv3E/download)

![](https://fh-aachen.sciebo.de/s/JkP1fcI9bXsI8pS/download)

![](https://fh-aachen.sciebo.de/s/XOwl9QyHqC6hTzb/download)
