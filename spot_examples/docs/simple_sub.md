# Spot Robot Interface Documentation

## Overview
As a developer using spot_ros2, it took me a lot of time to figure out how to use subscription from my local planner to send pose to the Spot robot. Therefore, I am providing an example of how to use subscriptions and actions together.

This implementation uses helper files `robot_commander` and `simple_spot_commander` to handle robot commands. The script listens to a ROS topic and converts received subscription messages into actions for the robot accordingly.

## Main Functionality

### Initialization
1. The `SpotRobotInterface` class provides an interface to control the Spot. It listens for pose commands and directs the robot accordingly. The interface ensures safe command execution by buffering messages when the robot is busy.
2. The script retrieves the robot name from the cli argument.
3. An instance of `RobotCommander` is created with the robot name.
4. The script logs the robot's name and initializes the robot.
5. If initialization fails, the script logs an error and exits.

### Topic Subscription and Action Execution
1. The script subscribes to the `/pose_commands` topic and listens for messages of type `Pose` using the `listen_to_pose_commands` of the `SpotRobotInterface`.
2. Subscription messages received on the topic are buffered and converted into actions for the robot using `process_message`. It sends the command to the spot and if any other message comes in, it buffers it.
3. The `process_message` uses `execute_command` to execute the action using the `robot_commander` instance.

### Message Processing
1. The script maintains a buffer (`latest_message`) for incoming messages and a flag (`is_busy`) to track execution status.
2. It listens to the `/pose_commands` topic using `Subscription(Pose, "/pose_commands")`.
3. If the robot is busy executing an action, incoming messages are buffered.
4. If the robot is not busy:
   - The robot executes the action based on the received `Pose` message using `robot_commander.walk_forward_with_vision_frame_goal(message)`.
   - Once execution is complete, `is_busy` is reset.
   - If a buffered message exists, it is processed next.
   - The script ensures sequential execution by waiting for each action to complete before sending the next `Pose` command.

## Dependencies
- ROS2
- synchros2
- geometry_msgs
- Boston Dynamics SDK

## Error Handling
- If the robot fails to initialize, a log message is generated, and the script exits.
- If the robot is busy, new messages are buffered instead of being lost.

## Execution
To run the script, execute:
```sh
ros2 run spot_examples simple_sub --robot <robot_name>
```

## Implementation Details

### SpotRobotInterface Class
- Initializes a `RobotCommander` instance.
- Subscribes to `/pose_commands` to receive pose messages.
- Buffers messages when busy and ensures sequential execution.

### RobotCommander Class
- Handles robot initialization and command execution.
- Converts received `Pose` messages into actions using Boston Dynamics' `SE3Pose` and `RobotCommandBuilder`.
- Uses `TFListenerWrapper` to transform poses into the robot's frame.

### SimpleSpotCommander Class
- Provides a ROS service interface for fundamental robot commands such as `claim`, `stand`, and `power_on`.
- Uses `synchros2` for ROS 2 service communication.

## Notes
- This script uses `synchros2` for ROS communication.
- It assumes that `RobotCommander` has a method `walk_forward_with_vision_frame_goal` to handle movement actions.
- The script ensures that actions are executed sequentially and prevents command loss by buffering messages when the robot is busy.

## Author
- Kabir Kedia <kabirk@andrew.cmu.edu>
