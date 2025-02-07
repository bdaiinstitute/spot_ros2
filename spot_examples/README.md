# spot_examples
This is a ROS 2 package that demonstrates how to use the Spot driver to control the robot.

Before running any of these nodes, make sure that the robot is in an open space and that you've built and sourced your ROS 2 workspace. 
```bash
cd <ros2_workspace>
colcon build --symlink-install
source install/setup.bash
```
After this, it's time to start the Spot driver. Define the environment variables `BOSDYN_CLIENT_USERNAME`, `BOSDYN_CLIENT_PASSWORD`, and `SPOT_IP` appropriately for your robot, or put the login credentials into a configuration file yaml ([example](../spot_driver/config/spot_ros_example.yaml)). The driver can be started via the following launchfile:
```bash
ros2 launch spot_driver spot_driver.launch.py
```
* If you are defining your login credentials in a configuration yaml instead of as environment variables, add the launch argument `config_file:=path/to/config.yaml`
* If you want to launch with a namespace, use the launch argument `spot_name:=<spot_name>`


## Nodes
Once the driver has been started, different examples can be run with the following command:
```bash
ros2 run spot_examples <example_node>
```
Follow the links on each of the node names for more detailed documentation about how each example works, possible command line arguments, and other safety considerations you may need to take into account. 
* [`walk_forward`](docs/walk_forward.md): A simple example that shows how to use ROS 2 to send `RobotCommand` goals to the Spot driver. If you are new to Spot and ROS 2, we recommend starting here.
* [`arm_simple`](docs/arm_simple.md): An example of converting the [BD Simple Arm Motion](https://dev.bostondynamics.com/python/examples/arm_simple/readme) example to use ROS 2. 
* [`send_inverse_kinematic_requests`](docs/send_inverse_kinematics_requests.md): An example that shows how to send inverse kinematics requests to the Spot Arm using ROS 2. 
* [`batch_trajectory`](docs/batch_trajectory.md): An example that shows how to send very long trajectories to Spot using ROS 2. 
* [`hello_spot`](docs/hello_spot.md): An example of converting the [BD Hello Spot](
https://dev.bostondynamics.com/python/examples/hello_spot/readme
) example to use ROS 2, demonstrating basic movement and image streaming. 
* [`arm_with_body_follow`](docs/arm_with_body_follow.md): An example that demonstrates simultaneous locomotion and manipulation using ROS 2.
* [`wasd`](docs/wasd.md): An example that offers basic teleoperation of Spot's locomotion and manipulation capabilities using ROS 2.

## Adding new examples
To add examples that demonstrate other features of the Spot driver, create a new node `new_example.py` in [spot_examples](spot_examples) and add it to `setup.py`. Make sure to also write a documentation file (`new_example.md`) in [docs](docs) that explains how to run the example and how the code works, and link to it in this central README. 
