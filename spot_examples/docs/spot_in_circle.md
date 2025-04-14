
## Overview

This script shows how to command a spot to move in a circular trajectory while optionally gazing at the center of the circle using the arm.

The implementation demonstrates coordinating Spot’s base movement and arm gaze commands simultaneously to achieve continuous motion with synchronized behavior.


It is built using the `synchros2` ROS 2 interface and uses `SimpleSpotCommander` along with `ActionClientWrapper` to send robot commands through ROS actions.

---

## Main Functionality

### Initialization

1. The `SpotInCircle` class is initialized using a CLI argument for robot name (`--robot <robot_namespace>`).
2. It creates:
   - Frame names (`body`, `odom`, `vision`) using `namespace_with`.
   - A TF listener (`TFListenerWrapper`) to perform frame transformations.
   - A Spot commander (`SimpleSpotCommander`) to send basic commands like `claim`, `stand`, and `power_on`.
   - An action client for `RobotCommand` actions (`ActionClientWrapper`).
3. The robot is initialized via:
   - `claim` → claims control of the robot.
   - `power_on` → turns the robot on.
   - `stand` → commands the robot to stand up.
4. If an arm is detected, the robot's arm is assumed to be active and used to point at a target (in this case, the center of the circular trajectory spot is moving in).

---

### Trajectory Planning and Execution

1. The method `get_me_a_circle(radius, steps)` computes a circular trajectory  as a list of relative base movements, which is divided into multiple linear + rotational steps:
   - The path is approximated using chords.
   - Each segment includes a forward motion (`dx`) and a fixed rotation (`dyaw`) relative to the robot's current position.
   - As the circular path is approximated using a series of straight-line segments followed by small rotations,  increasing the number of steps results in a smoother and more accurate circular trajectory.
   - A negative radius indicates **clockwise** movement; positive indicates **counter-clockwise**.

2. The method `spot_in_a_circle()` calls `get_me_a_circle()` to generate `(dx, dy, dyaw)` triplets and then:
   - If the arm is enabled:
     - Defines a gaze point `(0, radius, 0.1)` in the body frame. If the radius is negative, the arm will look toward its right (and make a clockwise circle when  used with base movements), and vice versa. 
     - Transforms the gaze point to the odometry frame using TF.
     - Sends an `arm_gaze_command` to keep the arm pointing at the center.
   - Sends `synchro_se2_trajectory_point_command` at each step to move the base.

3. After each movement, the robot is commanded to stop to ensure controlled transitions.

---

## Dependencies

- `ROS 2`
- `synchros2`
- `bosdyn.client` SDK
- `geometry_pb2`, `RobotCommandBuilder`
- `spot_msgs/action/RobotCommand`

---

## Error Handling

- If no ROS node is available (`ros_scope.node()` returns None), an exception is raised.
- If `claim`, `power_on`, or `stand` fail, the method logs an error and returns `False`.
- If the robot fails to stop after each move, an error is logged.

---

## Execution

To run the script:

```bash
ros2 run spot_examples spot_in_circle --robot <robot_name> --radius <radius> --steps <steps>
```

- The radius is the distance from the robot to the center of the circle (body frame) in meters. Negative sign indicates the robot is moving in a clockwise direction. Positive sign indicates the robot is moving in a counter-clockwise direction.
        

 - Steps is the number of steps to complete a full circle. More steps make the circle smoother, but the motion takes longer to complete. The angle subtended at the center by each step is (2.math(pi) / steps) radians.

## Author(s)

Mayank Mishra : <mayankmi@andrew.cmu.edu> 


Kabir Kedia : <kabirk@andrew.cmu.edu>
