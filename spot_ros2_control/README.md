# spot_ros2_control

This is a ROS 2 package designed to communicate with Spot's low level API through ROS 2 control. It sets up a hardware interface that can be either `mock` or connected to the robot via the `spot-cpp-sdk` (currently in progress). It also uses a standard joint state broadcaster and forward position controller provided from `ros2_control`.

## On-robot

Currently, the parameters for authenticating with your robot must be set in the environment variables `BOSDYN_CLIENT_USERNAME`, `BOSDYN_CLIENT_PASSWORD`, and `SPOT_IP`.

For a robot without an arm, run: `ros2 launch spot_ros2_control spot_ros2_control.launch.py hardware_interface:=spot-sdk`

For a robot with an arm, run: `ros2 launch spot_ros2_control spot_ros2_control.launch.py has_arm:=true controllers_config:=spot_controllers_with_arm.yaml hardware_interface:=spot-sdk`

This hardware interface currently does not accept commands but will stream the joint angles of the robot using the low level API at ~333 Hz. 


## Mock

To use a mock hardware interface, run the main `spot_ros2_control.launch.py` launchfile with the launch argument `hardware_interface:=mock`.

## Examples

Examples are provided to replicate [these joint control examples](https://github.com/boston-dynamics/spot-cpp-sdk/tree/master/cpp/examples/joint_control) from Boston Dynamics. They are currently only supported in `mock` mode, but are designed to show how you can send commands to the robot using the built in forward position controller.

Run the following commands to test these examples:

`ros2 launch spot_ros2_control noarm_squat.launch.py`

`ros2 launch spot_ros2_control wiggle_arm.launch.py`
