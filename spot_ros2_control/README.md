# spot_ros2_control

This is a ROS 2 package designed to communicate with Spot's low level API through ROS 2 control. It sets up a hardware interface that can be either `mock` or connected to the robot using the [Spot C++ SDK](https://github.com/boston-dynamics/spot-cpp-sdk). By default, it loads a standard joint state broadcaster and forward position controller provided from `ros2_control`.

## On-robot

If your parameters for authenticating with your robot are set in the environment variables `BOSDYN_CLIENT_USERNAME`, `BOSDYN_CLIENT_PASSWORD`, and `SPOT_IP`, run:

```bash
ros2 launch spot_ros2_control spot_ros2_control.launch.py hardware_interface:=robot
```

Login information can also be set in a configuration file, the same as in the `spot_driver` launchfile. For example, if you have a config file `spot_ros.yaml` containing the following information:
```
/**:
  ros__parameters:
    username: "username"
    password: "password"
    hostname: "00.00.00.00"
```
You can then run the launchfile with the following command:

```bash
ros2 launch spot_ros2_control spot_ros2_control.launch.py hardware_interface:=robot config_file:=path/to/spot_ros.yaml
```

If you wish to launch these nodes in a namespace, add the argument `spot_name:=<Robot Name>`.

This hardware interface will stream the joint angles of the robot using the low level API at 333 Hz onto the topic `/<Robot Name>/low_level/joint_states`.

Commands can be sent on the topic `/<Robot Name>/forward_position_controller/commands`. This will forward position commands directly to the spot sdk through the hardware interface. 

An alternative feed-forward controller provided by the `spot_controllers` package can be used to specify the position, velocity, and effort of all joints at the same time. To bring up this controller, add the launch argument `robot_controller:=forward_state_controller`. Commands can then be sent on the topic `/<Robot Name>/forward_state_controller/commands`. This controller expects the ordering of the command array to be `[<list of desired position>, <list of desired velocity>, <list of desired effort>]`.

> [!CAUTION]
> When using the forward position and state controllers, there is no safety mechanism in place to ensure smooth motion. The ordering of the command must match the ordering of the joints specified in the controller configuration file ([here for robots with an arm](config/spot_default_controllers_with_arm.yaml) or [here for robots without an arm](config/spot_default_controllers_without_arm.yaml)), and the robot can move in unpredictable and dangerous ways if this is not set correctly. Make sure to keep a safe distance from the robot when working with this controller and ensure the e-stop can easily be pressed if needed.

Additionally, the state publisher node, object synchronization node, and image publisher nodes from [`spot_driver`](../spot_driver/) will get launched by default when running on the robot to provide extra information such as odometry topics and camera feeds.
To turn off the image publishers (which can cause problems with bandwidth), add the launch argument `launch_image_publishers:=false`.

## Mock

To use a mock hardware interface, run:

```bash
ros2 launch spot_ros2_control spot_ros2_control.launch.py hardware_interface:=mock
```

By default, this will load a robot with no arm. If you want your mock robot to have an arm, add the launch argument `mock_arm:=True`. 

## Examples

Examples are provided to replicate [these joint control examples](https://github.com/boston-dynamics/spot-cpp-sdk/tree/master/cpp/examples/joint_control) from Boston Dynamics. They are currently only supported in `mock` mode, but are designed to show how you can send commands to the robot using the built in forward position controller.

Run the following commands to test these examples after launching `spot_ros2_control.launch.py`:
```bash
ros2 launch spot_ros2_control noarm_squat.launch.py
```
```bash
ros2 launch spot_ros2_control wiggle_arm.launch.py
```

## Additional Arguments
* `controllers_config`: If this argument is unset, a general purpose controller configuration will be loaded containing a forward position controller and a joint state publisher, that is filled appropriately based on whether or not the robot used (mock or real) has an arm. If you wish to load a different controller, this can be set here.
* `robot_controller`: This is the name of the robot controller that will be started when the launchfile is called. The default is the simple forward position controller. The name must match a controller in the `controllers_config` file.
* `launch_rviz`: If you do not want rviz to be launched, add the argument `launch_rviz:=False`.
