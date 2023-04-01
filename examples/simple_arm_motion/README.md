This is a simple example of using ROS2 to make the robot walk 1m forward.

## Running the Example

1.  Position the robot with 1m of clear space around it either sitting or standing (it will move its arm up and down)
2.  Make sure you've built and sourced your workspace:
    ```bash
    cd <ros2 workspace>
    colcon build --symlink-install
    source /opt/ros/humble/setup.bash
    source ./install/local_setup.bash
    ```

3.  Define the environment variables `BOSDYN_CLIENT_USERNAME`, `BOSDYN_CLIENT_PASSWORD`, and `SPOT_IP` appropriately for your robot.

4.  Start the driver:
```bash
ros2 launch spot_driver spot_driver.launch.py
```

5.  Run the example:
```bash
ros2 run simple_arm_motion arm_simple
```

The robot should move its arm up and down.


## Converting Direct API Calls to ROS2

While the driver provides plenty of helper services and topics, direct Spot API calls can also usually be replaced with ROS2 calls simply by converting the protobuf into a ROS message and using the `robot_command` action.  This example shows how to update [BD Simple Arm Motion](https://dev.bostondynamics.com/python/examples/arm_simple/readme) to use ROS2.  The same concepts can be applied to any code that uses the `RobotCommand` protobuf.

### Original Code

The original code that directly calls ROS2 is:
```python
# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Tutorial to show how to use Spot's arm.
"""
from __future__ import print_function

import argparse
import sys
import time

import bosdyn.api.gripper_command_pb2
import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.api import arm_command_pb2, geometry_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient


def hello_arm(config):
    """A simple example of using the Boston Dynamics API to command Spot's arm."""

    # See hello_spot.py for an explanation of these lines.
    bosdyn.client.util.setup_logging(config.verbose)

    sdk = bosdyn.client.create_standard_sdk('HelloSpotClient')
    robot = sdk.create_robot(config.hostname)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    assert robot.has_arm(), "Robot requires an arm to run this example."

    # Verify the robot is not estopped and that an external application has registered and holds
    # an estop endpoint.
    assert not robot.is_estopped(), "Robot is estopped. Please use an external E-Stop client, " \
                                    "such as the estop SDK example, to configure E-Stop."

    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
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

        # Move the arm to a spot in front of the robot, and open the gripper.

        # Make the arm pose RobotCommand
        # Build a position to move the arm to (in meters, relative to and expressed in the gravity aligned body frame).
        x = 0.75
        y = 0
        z = 0.25
        hand_ewrt_flat_body = geometry_pb2.Vec3(x=x, y=y, z=z)

        # Rotation as a quaternion
        qw = 1
        qx = 0
        qy = 0
        qz = 0
        flat_body_Q_hand = geometry_pb2.Quaternion(w=qw, x=qx, y=qy, z=qz)

        flat_body_T_hand = geometry_pb2.SE3Pose(position=hand_ewrt_flat_body,
                                                rotation=flat_body_Q_hand)

        robot_state = robot_state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)

        odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose.from_obj(flat_body_T_hand)

        # duration in seconds
        seconds = 2

        arm_command = RobotCommandBuilder.arm_pose_command(
            odom_T_hand.x, odom_T_hand.y, odom_T_hand.z, odom_T_hand.rot.w, odom_T_hand.rot.x,
            odom_T_hand.rot.y, odom_T_hand.rot.z, ODOM_FRAME_NAME, seconds)

        # Make the open gripper RobotCommand
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)

        # Combine the arm and gripper commands into one RobotCommand
        command = RobotCommandBuilder.build_synchro_command(gripper_command, arm_command)

        # Send the request
        cmd_id = command_client.robot_command(command)
        robot.logger.info('Moving arm to position 1.')

        # Wait until the arm arrives at the goal.
        block_until_arm_arrives_with_prints(robot, command_client, cmd_id)

        # Move the arm to a different position
        hand_ewrt_flat_body.z = 0

        flat_body_Q_hand.w = 0.707
        flat_body_Q_hand.x = 0.707
        flat_body_Q_hand.y = 0
        flat_body_Q_hand.z = 0

        flat_body_T_hand2 = geometry_pb2.SE3Pose(position=hand_ewrt_flat_body,
                                                 rotation=flat_body_Q_hand)
        odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose.from_obj(flat_body_T_hand2)

        arm_command = RobotCommandBuilder.arm_pose_command(
            odom_T_hand.x, odom_T_hand.y, odom_T_hand.z, odom_T_hand.rot.w, odom_T_hand.rot.x,
            odom_T_hand.rot.y, odom_T_hand.rot.z, ODOM_FRAME_NAME, seconds)

        # Close the gripper
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)

        # Build the proto
        command = RobotCommandBuilder.build_synchro_command(gripper_command, arm_command)

        # Send the request
        cmd_id = command_client.robot_command(command)
        robot.logger.info('Moving arm to position 2.')

        # Wait until the arm arrives at the goal.
        # Note: here we use the helper function provided by robot_command.
        block_until_arm_arrives(command_client, cmd_id)

        robot.logger.info('Done.')

        # Power the robot off. By specifying "cut_immediately=False", a safe power off command
        # is issued to the robot. This will attempt to sit the robot before powering off.
        robot.power_off(cut_immediately=False, timeout_sec=20)
        assert not robot.is_powered_on(), "Robot power off failed."
        robot.logger.info("Robot safely powered off.")


def block_until_arm_arrives_with_prints(robot, command_client, cmd_id):
    """Block until the arm arrives at the goal and print the distance remaining.
        Note: a version of this function is available as a helper in robot_command
        without the prints.
    """
    while True:
        feedback_resp = command_client.robot_command_feedback(cmd_id)
        robot.logger.info(
            'Distance to go: ' +
            '{:.2f} meters'.format(feedback_resp.feedback.synchronized_feedback.arm_command_feedback
                                   .arm_cartesian_feedback.measured_pos_distance_to_goal) +
            ', {:.2f} radians'.format(
                feedback_resp.feedback.synchronized_feedback.arm_command_feedback.
                arm_cartesian_feedback.measured_rot_distance_to_goal))

        if feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.status == arm_command_pb2.ArmCartesianCommand.Feedback.STATUS_TRAJECTORY_COMPLETE:
            robot.logger.info('Move complete.')
            break
        time.sleep(0.1)


def main(argv):
    """Command line interface."""
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args(argv)
    try:
        hello_arm(options)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.exception("Threw an exception")
        return False


if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
```

### Updating to ROS2

The first lines of the original code are:
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
If you want to ensure you only communicate with the robot via the ROS2 driver (which is not necessary, but may simplify your life and ensures all robot commands can be echoed on ROS topics, are caught in ros bags, etc), the best way to do so is to ensure that you never call `authenticate` in any other program.  That is done by the Spot driver, which should be the only piece of code that communicates with the robot.  All other programs communicate with the Spot driver.  Therefore, we replace the pieces of code that talk directly to Spot with their ROS counterparts:
```python
    node = Node('arm_simple')
    tf_listener = TFListenerWrapper('arm_simple_tf', wait_for_transform = [ODOM_FRAME_NAME,
                                                                           GRAV_ALIGNED_BODY_FRAME_NAME])

    robot = SpotCommander()
    robot_command_client = ActionClientWrapper(RobotCommand, 'robot_command')
```
This gives us four components, which we'll use in many ROS2 programs:
* A node: [ROS2 nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) handle publishing and subscribing on ROS2 topics.
* A TF listener: This handles computing transforms.  As we'll see later, it can be used in place of Spot API `RobotStateClient` to get information about where frames on the robot are.  For more information about ROS2 TF see [here](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html).  For more information about the TF wrapper and how we use it in these examples, see the [simple_walk_forward example](../simple_walk_forward/).
* A spot commander: This is a wrapper around service clients that call the spot driver to do simple things like get the lease and stand.  This is used in place of calls like `blocking_stand`.
* A robot command action client: This is the ROS2 action client that sends goals to the ROS2 action server (for more information about ROS2 actions see [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)).  This is used in place of the Spot API `RobotCommandClient`.  We use a wrapper around the built in ROS2 action client that allows us to wait for the goal to return without risk of deadlock.

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
The ROS2 code uses service calls to do the same thing.  The ROS2 stand service call also takes care of powering on the robot so we do not do that separately:
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

Once the robot is initialized and standing, the change to the code comes when we need to get the robot's current position.  With direct API calls, this uses the robot state:
```python
        robot_state = robot_state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
```
With ROS2, we use TF:
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
In ROS2, we convert the created protobuf to a ROS2 action goal we use the action client `send_goal_and_wait` function to replace the `block_until_arm_arrives_with_prints` function (we do not get the printing, but we could echo the action feedback topic or use the non-blocking `send_goal_async` if we wanted to do that):
```python
    # Convert to a ROS message
    action_goal = RobotCommand.Goal()
    conv.convert_proto_to_bosdyn_msgs_robot_command(command, action_goal.command)
    # Send the request and wait until the arm arrives at the goal
    node.get_logger().info('Moving arm to position 1.')
    robot_command_client.send_goal_and_wait(action_goal)
```

The last thing to note in this example is the line
```python
    tf_listener.shutdown()
```
Owing to a bug in how tf threads are handled, if you want programs involving the TF listener wrapper to end cleanly, you have to make this call before the `tf_listener` is destructed.