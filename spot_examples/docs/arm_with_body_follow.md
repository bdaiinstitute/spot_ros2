# arm_with_body_follow
An example that demonstrates simultaneous locomotion and manipulation using ROS 2.

## Running the Example
For this example, make sure to position the robot with 2m of clear space in front of the robot, either sitting or standing, as it will stand, move its arm forward, then walk forward. After the Spot driver is running, you can start the example with:
```bash
ros2 run spot_examples arm_with_body_follow
```
If you launched the driver with a namespace, use the following command instead:
```bash
ros2 run spot_examples arm_with_body_follow --robot <spot_name>
```
The robot will stand up, place its arm in front of itself, then slowly walk forward.

## Understanding the Code

Now let's go through [the code](../spot_examples/arm_with_body_follow.py) and see what's happening.

Similar to other examples, Hello Spot is designed as a class composed with a [ROS Node](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) that allows communication over [ROS topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html).

```python
class ArmWithBodyFollow:
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        self.node = ros_scope.node()
```
We first run `initialize_robot()`, which will claim then power on the robot:
```python
        result = self.robot.command("claim")
        ...
        result = self.robot.command("power_on")
```

Then we run `move()`, which will compose a synchronized command that allows simultaneous execution of mobility and manipulation commands.

To do this, we first craft a position for the arm to move to in front of the robot, using BD's `bodsyn.client.math_helpers` functions to assist in consecutive transformation composition: 
```python
        odom_T_body = self.tf_listener.lookup_a_tform_b(self.odom_frame_name, self.grav_aligned_body_frame_name)
        ...
        odom_T_hand = odom_T_body_se3 * math_helpers.SE3Pose.from_proto(body_T_hand)
```
then creating the arm_command:
```python
        arm_command = RobotCommandBuilder.arm_pose_command(
            odom_T_hand.x,
            odom_T_hand.y,
            odom_T_hand.z,
            odom_T_hand.rot.w,
            odom_T_hand.rot.x,
            odom_T_hand.rot.y,
            odom_T_hand.rot.z,
            ODOM_FRAME_NAME,
            seconds,
        )
```

as well as the `follow_arm_command()` that tells the robot's body to follow the arm. 
```python
        follow_arm_command = RobotCommandBuilder.follow_arm_command()
```
We build the synchro command, convert, and send the goal.
```python
        cmd = RobotCommandBuilder.build_synchro_command(follow_arm_command, arm_command)
        ...
        action_goal = RobotCommand.Goal()
        convert(cmd, action_goal.command)
        ...
        self.robot_command_client.send_goal_and_wait("arm_with_body_follow", action_goal)
```
This functionality is exposed to spot_ros2 thanks to our `convert()` function, which allows us to essentially send bosdyn protobuf commands using ROS 2.
