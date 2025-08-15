# spot_controllers

This is a ROS 2 package that provides custom ROS 2 controllers that can be used with [spot_ros2_control](../spot_ros2_control/).

This package contains two controllers: `spot_controllers/ForwardStateController` and `spot_controllers/SpotJointController` and one broadcaster: `spot_controllers/FootStateBroadcaster`.
Example configurations for setting up this controllers can be found in [`spot_ros2_control/config`](../spot_ros2_control/config/).

## `ForwardStateController`

This is a generic controller allows you to directly forward a set of commands over a set of interfaces.
It builds on top of ROS 2 control's [`forward_command_controller`](https://github.com/ros-controls/ros2_controllers/tree/master/forward_command_controller).
It is used with `spot_ros2_control` to forwad commands for position, velocity, and effort for all joints at the same time. 


## `SpotJointController`

This is a streaming controller specific to Spot's hardware interface, used to directly forward any subset of position, velocity, load, k_q_p, and k_qd_p over any subset of Spot's joints.
It receives commands of type `spot_msgs/msg/JointCommand`, which is almost the same as a `sensor_msgs/msg/JointState` message but with extra fields for Spot's `k_q_p` and `k_qd_p` command interfaces.
This controller claims all command interfaces of the robot on activation, but your `JointCommands` do not have to contain commands for all interfaces -- they can just contain a subset.
The `name` field in this topic should contain the names of the joints you wish to control, and for each entry in `name`, you can fill one or more of the elements in `postition`, `velocity`, `effort`, `k_q_p`, or `k_qd_p`.
Any interfaces that are not specified in this message will be left unchanged at their last commanded value.

For example, to send one position command to one joint, the joint command would be formed such as:
```
name: ["joint_1"]
postion: [1.0]
velocity: []
effort: []
k_q_p: []
k_qd_p: []
```
Or to control position, velocity, and k_qd_p of 3 different joints:
```
name: ["joint_1", "joint_2", "joint_3"]
postion: [1.0, 2.0, 3.0]
velocity: []
effort: [4.0, 5.0, 6.0]
k_q_p: []
k_qd_p: [7.0, 8.0, 9.0]
```

## `FootStateBroadcaster`
This is a broadcaster that will publish the feet contact states of Spot onto a topic `spot_msgs/msg/FootStateArray`.
Each message contains four `FootState` messages filled in with the appropriate `contact` field, that correspond in order to the contacts for the front left, front right, rear left, and rear right feet.
0 corresponds to an unknown contact, 1 corresponds to the foot being in contact in the floor, and 2 corresponds to the foot not being in contact with the floor.
Note that in contrast to the `FootState` published by the high level driver, Spot's streaming interface for foot contacts does NOT contain an estimate for the foot's position with respect to the body.
Therefore, the `foot_position_rt_body` field of the `FootState` message is not filled in and should be disregarded.

## `SpotPoseBroadcaster`
This is a broadcaster that will publish the estimates of the `odom` and `vision` frame transforms to `body` as both a TF frame and a ROS pose message.
This broadcaster reads from the state interfaces `vision_t_body` and `odom_t_body`.
It broadcasts these to TF frames `low_level/vision_t_body` and `low_level/odom_t_body` (named this way as to not conflict with the existing `odom` and `vision` frames published from the high level driver estimates).
Note that these frames are broadcasted to the TF tree as the children of the `body` frame to ensure that `body` does not contain more than one parent.
The same data as a `Pose` message is also available on the topics `vision_t_body` and `odom_t_body`. 
These directly publish the transform of the body frame with respect to `vision`/`odom`.
