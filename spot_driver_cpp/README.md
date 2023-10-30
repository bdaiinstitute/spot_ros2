# spot_driver_cpp

This package provides ROS 2 nodes to interact with Spot through its C++ SDK.

## Why C++?

The Spot C++ SDK is more performant than the Spot Python SDK for some operations.

For example, the Spot C++ SDK's image client can retrieve images from all Spot's RGB and depth cameras at close to their native refresh rate of 15Hz.

In constrast, the Spot Python SDK's image client slows substantially when requesting images from multiple depth cameras at once, and it can only maintain around 1Hz when requesting images from all RGB and depth cameras at once.
