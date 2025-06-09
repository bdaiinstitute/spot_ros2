# walk_forward
This is a simple example of using ROS 2 to make the robot walk 1m forward.

## Running the Example
For this example, make sure to position the robot with 2m of clear space in front of it either sitting or standing (it's going to walk 1m forward). After the Spot driver is running, you can start the example with:
```bash
ros2 run spot_examples walk_forward
```
If you launched the driver with a namespace, use the following command instead:
```bash
ros2 run spot_examples walk_forward --robot <spot_name>
```
The robot should walk forward.

## Understanding the Code

Now let's go through [the code](../spot_examples/walk_forward.py) and see what's happening.

Because ROS generally requires persistent things like publishers and subscribers, it’s often useful to have a class around everything.  A [ROS Node](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) is an object that can interact with [ROS topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) so our classes usually inherit from Node or contain a node:
```python
class WalkForward:
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None):
        super().__init__('walk_forward')
        node = node or ros_scope.node()
```
Note here we default to the [current scope](https://github.com/bdaiinstitute/ros_utilities/blob/main/synchros2/synchros2/scope.py) node if none is provided.

Then we set up ROS's [TF](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html), which helps us transform between robot frames:
```python
         self._tf_listener = TFListenerWrapper(node)
         self._tf_listener.wait_for_a_tform_b(BODY_FRAME_NAME, VISION_FRAME_NAME)
```
We use a [wrapper](https://github.com/bdaiinstitute/ros_utilities/blob/main/synchros2/synchros2/tf_listener_wrapper.py) that supports synchronous operation around ROS 2’s asynchronous [TF implementation](https://github.com/ros2/rclpy/tree/humble).  Passing it the body and vision frame names causes the wrapper to wait until it sees those frames.  This lets us make sure the robot is started and TF is working before proceeeding.

In order to perform small actions with the robot we use the [SimpleSpotCommander class](../../utilities/utilities/simple_spot_commander.py).  This wraps some service clients that talk to services offered by the spot driver.
```python
        self._robot = SimpleSpotCommander(self._robot_name, node)
```

Finally we want to be able to command Spot to do things.  We do this via a [wrapper](https://github.com/bdaiinstitute/ros_utilities/blob/main/synchros2/synchros2/action_client.py) around the action client that talks to an action server running in the Spot driver (for more information about ROS 2 actions see [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)):
```python
        self._robot_command_client = ActionClientWrapper(RobotCommand, 'robot_command', node)
```
The wrapper we use here automatically waits for the action server to become available during construction.  It also offers the `send_goal_and_wait` function that we’ll use later.

Before we can walk around, we need to claim the robot’s lease, power it on, and stand it up.  We can do all of these via the spot commander:
```python
    def initialize_robot(self):
        self._logger.info('Claiming robot')
        result = self._robot.command('claim')
        if not result.success:
            self._logger.error('Unable to claim robot message was ' + result.message)
            return False
        self._logger.info('Claimed robot')

        # Stand the robot up.
        self._logger.info('Powering robot on')
        result = self._robot.command('power_on')
        if not result.success:
            self._logger.error('Unable to power on robot message was ' + result.message)
            return False
        self._logger.info('Standing robot up')
        result = self._robot.command('stand')
        if not result.success:
            self._logger.error('Robot did not stand message was ' + result.message)
            return False
        self._logger.info('Successfully stood up.')
        return True
```

Now we’re ready to walk the robot around!
```python
    def walk_forward_with_world_frame_goal(self):
        self._logger.info('Walking forward using a world frame goal')
        world_t_robot = self._tf_listener.lookup_a_tform_b(VISION_FRAME_NAME,
                                                           BODY_FRAME_NAME).get_closest_se2_transform()
        world_t_goal = world_t_robot * ROBOT_T_GOAL
```
Goals need to be sent in a static world frame (at least in v3.3…) but we have an offset in the body frame.  Therefore, we first look up the robot’s current position in the world using TF.  We then multiply by the offset we want in the robot frame.  Note that "vision" is Spot’s name for the static world frame it updates using visual odometry.  See [here](https://dev.bostondynamics.com/docs/concepts/geometry_and_frames) for more about Spot’s frames.

Now we construct a goal to send to the Spot driver:
```python
        proto_goal = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=world_t_goal.x, goal_y=world_t_goal.y, goal_heading=world_t_goal.angle,
            frame_name=VISION_FRAME_NAME)
        action_goal = RobotCommand.Goal()
        convert(proto_goal, action_goal.command)
```
This constructs a protobuf message using the BD built in tools and converts it into a ROS action goal of a type that can be sent by our action client.

Finally we send the goal to the Spot driver:
```python
        self._robot_command_client.send_goal_and_wait(action_goal)
```
Note that `send_goal_and_wait` is a function of our [ActionClientWrapper](https://github.com/bdaiinstitute/ros_utilities/blob/main/synchros2/synchros2/action_client.py) and not a built in ROS 2 function.
