# spot_driver

This package provides the central ROS 2 interface to interact with Spot.

The primary launchfile in this package is `spot_driver.launch.py`. Once you've built and sourced your workspace, launch it with:

`ros2 launch spot_driver spot_driver.launch.py`

* Various driver parameters can be customized using a configuration file -- see [spot_ros_example.yaml](config/spot_ros_example.yaml) for reference. To launch the driver with a configuration file, add the launch argument `config_file:=path/to/config.yaml`. Note that you can specify the hostname, username, and password for login to the robot in this file, or they can alternatively be set in the environment variables `SPOT_IP`, `BOSDYN_CLIENT_USERNAME`, and `BOSDYN_CLIENT_PASSWORD`, respectively. 
* To launch the process within a namespace, add the launch argument `spot_name:={name}`
* To visualize Spot in RViz, add the launch argument `launch_rviz:=True`. This will automatically generate the appropriate RViz config file for your robot's name using `rviz.launch.py`.
* To publish point clouds, add the launch argument `publish_point_clouds:=True`. This is disabled by default.

The Spot driver contains both Python and C++ nodes. Spot's Python SDK is used for many operations. For example, `spot_ros2` is the primary node that connects with Spot and creates the ROS 2 action servers and services. Spot's C++ SDK is used in nodes like `spot_image_publisher_node` to retrieve images from Spot's RGB and depth cameras at close to their native refresh rate of 15 Hz -- something that is not possible using the Python SDK. 

## Examples
For some examples of using the Spot ROS 2 driver, check out [`spot_examples`](../spot_examples/).
