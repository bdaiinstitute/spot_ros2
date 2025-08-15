# arm_simple
This is a simple example of using ROS 2 to move Spot's arm up and down and open and close the gripper.

## Running the Example
For this example, make sure to position the robot with 1m of clear space around it, either sitting or standing (it will move its arm up and down). After the Spot driver is running, you can start the example with:
```bash
ros2 run spot_examples arm_simple
```
If you launched the `spot_ros2` driver with a namespace, use the following command instead:
```bash
ros2 run spot_examples arm_simple --robot <spot_name>
```

## Converting Direct API Calls to ROS 2

While the driver provides plenty of helper services and topics, direct Spot API calls can also usually be replaced with ROS 2 calls simply by converting the protobuf into a ROS message and using the `robot_command` action.  This example shows how to update the [BD Simple Arm Motion example](https://dev.bostondynamics.com/python/examples/arm_simple/readme) to use ROS 2 instead of direct API calls.  The same concepts can be applied to any code that uses the `RobotCommand` protobuf.

We'll go through the changes the [ROS 2 code](../spot_examples/arm_simple.py) makes to the [original example provided by Boston Dynamics](https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/arm_simple/arm_simple.py).

The first substantive lines of the original code are:
```python
def hello_arm(config):
    """A simple example of using the Boston Dynamics API to command Spot's arm."""

    # See hello_spot.py for an explanation of these lines.
    bosdyn.client.util.setup_logging(config.verbose)

    sdk = bosdyn.client.create_standard_sdk('HelloSpotClient')
    robot = sdk.create_robot(config.hostname)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()
```
If you want to ensure you only communicate with the robot via the ROS 2 driver (which is not necessary, but may simplify your life and ensures all robot commands can be echoed on ROS topics, are caught in ros bags, etc), the best way to do so is to ensure that you never call `authenticate` in any other program.  That is done by the Spot driver, which should be the only piece of code that communicates with the robot.  All other programs communicate with the Spot driver via ROS 2.  Therefore, we replace the pieces of code that talk directly to Spot with their ROS counterparts:
```python
    node = ros_scope.node()

    tf_listener = TFListenerWrapper(node)
    tf_listener.wait_for_a_tform_b(ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)

    robot = SimpleSpotCommander()
    robot_command_client = ActionClientWrapper(RobotCommand, 'robot_command', node)
```
This gives us four components, which we'll use in many ROS 2 programs:
* A node: [ROS 2 nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) are the objects that interact with [ROS 2 topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) and almost all ROS programs require them. Here we fetch the one in scope, already instantiated and serviced in the background. Had we instantiated a node of our own, we would have had to spin it ourselves.
* A TF listener: This handles computing transforms.  As we'll see later, it can be used in place of Spot API `RobotStateClient` to get information about where frames on the robot are.  For more information about ROS 2 TF see [here](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html).  For more information about our [TF wrapper](https://github.com/bdaiinstitute/ros_utilities/blob/main/synchros2/synchros2/tf_listener_wrapper.py) and how we use it in these examples, see the [simple_walk_forward example](../simple_walk_forward/).
* A spot commander: This is a [wrapper](../utilities/utilities/simple_spot_commander.py) around service clients that call the spot driver to do simple things like get the lease and stand.  This is used in place of calls like `blocking_stand`.
* A robot command action client: This is the ROS 2 action client that sends goals to the ROS 2 action server (for more information about ROS 2 actions see [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)).  This is used in place of the Spot API `RobotCommandClient`.  We use a [wrapper](https://github.com/bdaiinstitute/ros_utilities/blob/main/synchros2/synchros2/action_client.py) around the built in ROS 2 action client that allows us to wait for the goal to return without risk of deadlock.

The original code uses direct calls to the API to power on and stand the robot:
```python
        # Now, we are ready to power on the robot. This call will block until the power
        # is on. Commands would fail if this did not happen. We can also check that the robot is
        # powered at any point.
        robot.logger.info("Powering on robot... This may take a several seconds.")
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), "Robot power on failed."
        robot.logger.info("Robot powered on.")

        # Tell the robot to stand up. The command service is used to issue commands to a robot.
        # The set of valid commands for a robot depends on hardware configuration. See
        # RobotCommandBuilder for more detailed examples on command building. The robot
        # command service requires timesync between the robot and the client.
        robot.logger.info("Commanding robot to stand...")
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        blocking_stand(command_client, timeout_sec=10)
        robot.logger.info("Robot standing.")
```
The ROS 2 code uses service calls to do the same thing:
```python
    # Claim robot
    node.get_logger().info('Claiming robot')
    result = robot.command('claim')
    if not result.success:
        node.get_logger().error('Unable to claim robot message was ' + result.message)
        return False
    node.get_logger().info('Claimed robot')

    # Stand the robot up.
    node.get_logger().info('Powering robot on')
    result = robot.command('power_on')
    if not result.success:
        node.get_logger().error('Unable to power on robot message was ' + result.message)
        return False
    node.get_logger().info('Standing robot up')
    result = robot.command('stand')
    if not result.success:
        node.get_logger().error('Robot did not stand message was ' + result.message)
        return False
    node.get_logger().info('Successfully stood up.')
```
Note that we also use ROS's text logger instead of the Spot API logger.

Once the robot is initialized and standing, the next change to the code comes when we need to get the robot's current position.  With direct API calls, this uses the robot state:
```python
        robot_state = robot_state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
```
With ROS 2, we use TF:
```python
    odom_T_flat_body = tf_listener.lookup_a_tform_b(ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
```

The commands are built the same way after this, but we change how we send them.  The Spot API code has a function for waiting until the arm has moved to position:
```python
        # Send the request
        cmd_id = command_client.robot_command(command)
        robot.logger.info('Moving arm to position 1.')

        # Wait until the arm arrives at the goal.
        block_until_arm_arrives_with_prints(robot, command_client, cmd_id)
```
In ROS 2, we convert the created protobuf to a ROS 2 action goal and we use the action client wrapper `send_goal_and_wait` function to replace the `block_until_arm_arrives_with_prints` function (we do not get the printing, but we could echo the action feedback topic or use the non-blocking `send_goal_async` if we wanted to do that):
```python
    # Convert to a ROS message
    action_goal = RobotCommand.Goal()
    convert(command, action_goal.command)
    # Send the request and wait until the arm arrives at the goal
    node.get_logger().info('Moving arm to position 1.')
    robot_command_client.send_goal_and_wait(action_goal)
```

The last thing to note in this example is the line
```python
    tf_listener.shutdown()
```
