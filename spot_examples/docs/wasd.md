# wasd
An example that offers basic teleoperation of Spot's locomotion and manipulation capabilities using ROS 2.

## Running the Example
For this example, make sure to position the robot with 2m of clear space on all sides, as you will be able to command walking in translation and rotation and arm stowing and unstowing.
```bash
ros2 run spot_examples wasd
```
If you launched the driver with a namespace, use the following command instead:
```bash
ros2 run spot_examples wasd --robot <spot_name>
```
The robot will stand up, and will execute any commands you send through the curses (text-based) interface in your terminal.


*Keybinds*

Following is a list of keyboard keys and the robot functions to which they map.

    esc : make the robot stop executing all running commands
    tab : sit and quit example
      p : toggle power
      f : stand
      v : sit
      w : move forward
      s : move backward
      a : move left
      d : move right
      q : rotate (yaw) left
      e : rotate (yaw) right
      u : unstow arm
      j : stow arm
      b : battery change pose
      r : self-right (after battery change)



## Understanding the Code

Now let's go through [the code](../spot_examples/arm_with_body_follow.py) and see what's happening.

Similar to other examples, Hello Spot is designed as a class composed with a [ROS Node](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) that allows communication over [ROS topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html).

```python
class WasdInterface:
    ...
    def __init__(self, robot_name: Optional[str] = None) -> None:
        self.node = ros_scope.node()
```
In the `__init__()` function of the `WasdInterface` class, we create a `_command_dictionary` member, which maps keypresses to member functions that command functions of the robot. For example, a key press on key "v" will call the `_sit()` member function:
```python
        self._command_dictionary = {
            ...
            ord("v"): self._sit,
            ...
        }
```
    
The function `_sit()`, in turn, calls the robot's `/sit` service, by sending an asynchronous call of ROS 2 message type `Trigger` through the sit service client `self.cli_sit`.
```python
    def _sit(self) -> None:
        self.cli_sit.call_async(Trigger.Request())
```

The sit service client, `self.cli_sit`, was instantiated in the constructor of WasdInterface along with several other service clients for services such as stand, power on, and stow arm 
```python
        self.cli_sit = self.node.create_client(Trigger, namespace_with(robot_name, "sit"))
        self.cli_stand = self.node.create_client(Trigger, namespace_with(robot_name, "stand"))
        ...
        self.cli_power_on = self.node.create_client(Trigger, namespace_with(robot_name, "power_on"))
        ...
        self.cli_stow = self.node.create_client(Trigger, namespace_with(robot_name, "arm_stow"))
```
each, of which, had assigned a corresponding keymapping in `_command_dictionary` for its member function
```python
            ord("f"): self._stand,
            ...
            ord("p"): self._toggle_power,
            ...
            ord("j"): self._stow,

```

To command a forward movement, the "w" key is mapped to the `_move_forward()` member function,
```python 
            ord("w"): self._move_forward,
```
which, in turn, calls the helper function `_velocity_cmd_helper()` with details about the direction and magnitude of the velocity requested
```python
    def _move_forward(self) -> None:
        self._velocity_cmd_helper("move_forward", v_x=VELOCITY_BASE_SPEED)
```

The `_velocity_cmd_helper()` function takes 2D holonomic velocity requests (x, y translation and z rotation), crafts a ROS 2 `geometry_msgs/msg/Twist` message, and publishes to the robot's `/cmd_vel` topic.
```python
    def _velocity_cmd_helper(self, desc: str = "", v_x: float = 0.0, v_y: float = 0.0, v_rot: float = 0.0) -> None:
        twist = Twist()
        twist.linear.x = v_x
        twist.linear.y = v_y
        twist.angular.z = v_rot
        start_time = time.time()
        while time.time() - start_time < VELOCITY_CMD_DURATION:
            self.pub_cmd_vel.publish(twist)
            time.sleep(0.01)
        self.pub_cmd_vel.publish(Twist())
```
As shown above, we don't send this cmd_vel topic once; rather, we continuously send it unti VELOCITY_CMD_DURATION, at which point we send a default `Twist` message, where all velocity fields default to 0, stopping the robot.

Lasty, we use ROS 2 subscribers to obtain and display information about the robot's state. The current battery state, for example, which includes battery state-of-charge percentage and charge/discharge state, is stored in `WasdInterface`'s `latest_battery_status` member, 
```python
        self.latest_battery_status: Optional[BatteryStateArray] = None
```
which is updated every time the battery state ROS 2 subscriber sees a message published by the robot on the `/<robot_name>/status/battery_states` topic
```python
        self.sub_battery_state = self.node.create_subscription(
            BatteryStateArray, namespace_with(robot_name, "status/battery_states"), self._status_battery_callback, 1
        )
```
and calls `_status_battery_callback()`
```python
    def _status_battery_callback(self, msg: BatteryState) -> None:
        self.latest_battery_status = msg
```

At each 'tick' of the curses UI refresh, we use `_battery_str()` to return a formatted string represntation of `self.latest_battery_status` to be printed on the screen.
