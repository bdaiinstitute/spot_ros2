# hello_spot
An example of converting the [BD Hello Spot](
https://dev.bostondynamics.com/python/examples/hello_spot/readme) example to use ROS 2, demonstrating basic movement and image streaming.

## Running the Example
For this example, make sure to position the robot with 0.5m of clear space on all sides, either sitting or standing, as it will twist and move in place. After the Spot driver is running, you can start the example with:
```bash
ros2 run spot_examples hello_spot
```
If you launched the driver with a namespace, use the following command instead:
```bash
ros2 run spot_examples hello_spot --robot <spot_name>
```
The robot should stand (if not already), then twist its body along the yaw axis, then simultaneously bow and take a photo from its front left camera, which will be displayed in a pop up and saved locally on the PC from which spot_driver is run.

## Understanding the Code

Now let's go through [the code](../spot_examples/hello_spot.py) and see what's happening.


Similar to other examples, Hello Spot is designed as a class composed with a [ROS Node](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) that allows communication over [ROS topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html).
```python
class HelloSpot:
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        self.node = ros_scope.node()
```
In this example, we will create a ROS 2 subscriber to listen to the images Spot takes from its front left camera, which it publishes to the `/<robot_name>/camera/frontleft/image` topic.
```python
        self.image_sub = self.node.create_subscription(
            Image, namespace_with(robot_name, "camera/frontleft/image"), self.image_callback, 10
        )
```


We then instantiate both SimpleSpotCommander() and ActionClientWrapper() as ways to send both simple and more complex commands to Spot, respectively.
```python
        self.robot = SimpleSpotCommander(self._robot_name, node)
        self.robot_command_client = ActionClientWrapper(RobotCommand, 'robot_command', node)
```
The wrapper we use here automatically waits for the action server to become available during construction.  It also offers the `send_goal_and_wait` function that we’ll use later.


In `initialize_robot()` we first claim the lease of the robot using SimpleSpotCommander,
```python
        result = self.robot.command("claim")
```

then power it on
```python
        result = self.robot.command("power_on")
```

Our first commanded movement, `stand_default()`, is a simple stand: 
```python
        result = self.robot.command("stand")
```
Our second movement, `stand_twisted()`, will request a stand with a 0.4 radian twist in yaw. For this more complicated movement, we will use the ActionClientWrapper we instantiated, as it will allow us to send more complex RobotCommand goals. We start by creating the SO(3) rotation matrix detailing this twist:
```python
        footprint_R_body = bosdyn.geometry.EulerZXY(yaw=0.4, roll=0.0, pitch=0.0)
```
then feed this rotation as an argument to our RobotCommandGoal
```python
        cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)

```
which is converted into a ROS 2 action goal, and synchronously sent to the robot: 
```python
        action_goal = RobotCommand.Goal()
        convert(cmd, action_goal.command)
        self.logger.info("Twisting robot")
        self.robot_command_client.send_goal_and_wait("twisting_robot", action_goal)
```

Our final movement, `stand_3_pt_traj()` will have the robot follow a standing trajectory through thre points, resembling a bow. 

We first obtain the robot's current "world" frame to robot body frame transformation, which we will use as a starting point relative to which the next body positions will be based.
```python
        odom_T_flat_body = self.tf_listener.lookup_a_tform_b(self.odom_frame_name, self.grav_aligned_body_frame_name)
```
Note: The "odom" frame is the most basic "world frame" proxy provided by BD's spot-sdk, and the "grav aligned body frame" represents the robot's body's current 7 DOF pose (orientation + position), but with the Z axis fixed vertically.

Next, we create the transformations for the remaining 3 body pose "keyframes"
```python
        flat_body_T_pose1 = math_helpers.SE3Pose(x=0.075, y=0, z=0, rot=math_helpers.Quat())
        flat_body_T_pose2 = math_helpers.SE3Pose(x=0.0, y=0, z=0, rot=math_helpers.Quat(w=0.9848, x=0, y=0.1736, z=0))
        flat_body_T_pose3 = math_helpers.SE3Pose(x=0.0, y=0, z=0, rot=math_helpers.Quat())
```

convert each of these into protobuf form
```python
        traj_point1 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body_se3 * flat_body_T_pose1).to_proto(), time_since_reference=seconds_to_duration(t1)
        )
        traj_point2 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body_se3 * flat_body_T_pose2).to_proto(), time_since_reference=seconds_to_duration(t2)
        )
        traj_point3 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body_se3 * flat_body_T_pose3).to_proto(), time_since_reference=seconds_to_duration(t3)
        )
```
assemble into a single protobuf that is converted to a ROS 2 goal and synchronously sent to the robot
```python
        traj = trajectory_pb2.SE3Trajectory(points=[traj_point1, traj_point2, traj_point3])

        body_control = spot_command_pb2.BodyControlParams(
            body_pose=spot_command_pb2.BodyControlParams.BodyPose(
                root_frame_name=ODOM_FRAME_NAME, base_offset_rt_root=traj
            )
        )

        mobility_params = spot_command_pb2.MobilityParams(body_control=body_control)

        stand_command = RobotCommandBuilder.synchro_stand_command(params=mobility_params)

        stand_command_goal = RobotCommand.Goal()
        convert(stand_command, stand_command_goal.command)
        self.logger.info("Beginning absolute body control while standing.")
        self.robot_command_client.send_goal_and_wait(action_name="hello_spot", goal=stand_command_goal, timeout_sec=10)
```


Lastly, we call `_maybe_display_image()` and `_maybe_save_image()`, which display and save the latest image message stored in `self.latest_image_raw`. 
```python
        self.logger.info("Displaying image.")
        self._maybe_display_image()
        self._maybe_save_image()
```
`self.latest_image_raw` is updated every time the spot_driver publishes a new image to the `/<robot_name>/camera/frontleft/image` topic, thanks to the `image_sub` and `image_callback()`
```python
        self.image_sub = self.node.create_subscription(
            Image, namespace_with(robot_name, "camera/frontleft/image"), self.image_callback, 10
        )
```
