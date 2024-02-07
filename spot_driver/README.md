# spot_driver

This package provides ROS 2 nodes to interact with Spot through its SDK.

## Why Python?
Being known for it's ease of use and flexibility, the Spot Python SDK is used for many operations.
There are several entry points, though it is recommended to use the provided launch files.

- `spot_ros2` is the node used to connect to the spot and creates ROS 2 action servers and services.
- `command_spot` provides a command line interface with ROS 2 clients to send simple commands.
- `spot_publish_cameras` publishes camera images from spot cameras to ROS topics.
> :warning: **Warning:** For performance reasons, `spot_publish_cameras` is being rewritten in C++.

## Why C++?

The Spot C++ SDK is more performant than the Spot Python SDK for some operations.

For example, the Spot C++ SDK's image client can retrieve images from all Spot's RGB and depth cameras at close to their native refresh rate of 15Hz.

In constrast, the Spot Python SDK's image client slows substantially when requesting images from multiple depth cameras at once, and it can only maintain around 1Hz when requesting images from all RGB and depth cameras at once.
