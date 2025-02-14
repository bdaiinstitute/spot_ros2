# Spot Robot Interface Documentation

## Overview
As a developer using spot_ros2, it tooke me a lot of time to figure out how to use subscription from my local planner to send pose to the spot. Therefore I am providing an example as to how to use subscriptions and actions together. 


It utilizes helper files `robot_commander` and `simple_spot_commander` to handle robot commands. The script listens to a ROS topic and converts received subscription messages into actions for the robot accordingly.

## Environment Variables
- `ROBOT_NAME`: The name of the robot is retrieved from the environment variable.

## Main Functionality
### Initialization
1. The `main()` function is the entry point of the script.
2. The script retrieves the robot name from the environment variable.
3. An instance of `RobotCommander` is created with the robot name.
4. A ROS parameter `config_file_path` is declared and retrieved.
5. The configuration file specified by `config_file_path` is loaded.
6. The script logs the robot's name and initializes the robot.
7. If initialization fails, the script logs an error and exits.

### Topic Subscription and Action Execution
1. The `spot_controller_topic` is extracted from the configuration file.
2. The script subscribes to the topic and listens for messages of type `Pose`.
3. If debugging is enabled, the script logs the topic name.
4. Subscription messages received on the topic are buffered and converted into actions for the robot.
5. The script waits for the current action to complete before sending the next `Pose` command.

### Message Processing
1. The script maintains a buffer (`latest_message`) for incoming messages and a flag (`is_busy`) to track the execution status.
2. It listens to the `spot_controller_topic` using `Subscription(Pose, spot_controller_topic)`.
3. If the robot is busy executing an action, incoming messages are buffered.
4. If the robot is not busy:
   - The robot executes the action based on the received `Pose` message using `robotcommander.walk_forward_with_world_frame_goal(message)`.
   - Once execution is complete, `is_busy` is reset.
   - If a buffered message exists, it is processed next.
   - The script ensures sequential execution by waiting for each action to complete before sending the next `Pose` command.

## Error Handling
- If the robot fails to initialize, a log message is generated, and the script exits.
- If the robot is busy, new messages are buffered instead of being lost.

## Execution
To run the script, execute:
```sh
ros2 run spot_examples simple_sub
```
Make sure the required environment variable `ROBOT_NAME` is set and the configuration file is accessible.

## Notes
- This script uses `synchros2` for ROS communication.
- It assumes that `RobotCommander` has a method `walk_forward_with_world_frame_goal` to handle movement actions.
- The script ensures that actions are executed sequentially and prevents command loss by buffering messages when the robot is busy.

## Author
- Kabir Kedia <kabirk@andrew.cmu.edu>

