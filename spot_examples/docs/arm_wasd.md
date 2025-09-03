# WASD Robot Control Interface Documentation

## Table of Contents
1. [Running the Example](#running-the-example)
2. [Understanding the Code](#understanding-the-code)

---

## Running the Example
For this example, make sure to position the robot with 2m of clear space on all sides, as you will be able to command walking in translation and rotation and arm stowing and unstowing.
```bash
ros2 run spot_examples arm_wasd
```
If you launched the driver with a namespace, use the following command instead:
```bash
ros2 run spot_examples arm_wasd --robot <spot_name>
```

#### Initial Robot Setup

The interface will automatically:
1. Claim the robot
2. Power on the robot
3. Wait for user commands

#### Control Modes

The interface has two control modes that you can toggle between:

**BASE MOVEMENT MODE** (default):
- Controls the robot's body movement
- Walking, turning, standing, sitting

**ARM CONTROL MODE**:
- Controls the robot's arm in 6 degrees of freedom
- Arm positioning and end-effector rotation

### Key Controls

#### Mode Switching
- **[T]**: Toggle between BASE and ARM control modes
- **[Tab]**: Quit the program
- **[ESC]**: Emergency stop

#### Basic Robot Controls (Available in Both Modes)
- **[p]**: Toggle power on/off
- **[f]**: Stand up
- **[v]**: Sit down
- **[r]**: Self-right (if robot is on its side)
- **[b]**: Battery change pose (rollover)

#### Arm Controls (Available in Both Modes)
- **[y]**: Unstow arm (deploy from stowed position)
- **[h]**: Stow arm (return to stowed position)
- **[n]**: Open gripper
- **[m]**: Close gripper

#### Base Movement Controls (BASE MODE only)
- **[w]**: Move forward
- **[s]**: Move backward
- **[a]**: Strafe left
- **[d]**: Strafe right
- **[q]**: Turn left
- **[e]**: Turn right

#### Arm Movement Controls (ARM MODE only)
- **[w]**: Move arm forward/out
- **[s]**: Move arm backward/in
- **[a]**: Rotate arm counter-clockwise
- **[d]**: Rotate arm clockwise
- **[q]**: Move arm up
- **[e]**: Move arm down

#### End-Effector Rotation (ARM MODE only)
- **[i/k]**: Rotate around Y-axis (+/-)
- **[u/o]**: Rotate around X-axis (+/-)
- **[j/l]**: Rotate around Z-axis (+/-)

### Safety Features

#### Automatic Safety Measures
- **Emergency stop**: ESC key stops all movement immediately
- **Arm locking**: When arm is unstowed and base commands are issued, arm locks in position
- **Service call blocking**: Prevents conflicting commands during service calls
- **Graceful shutdown**: On exit, automatically sits robot and powers down

#### Best Practices
1. **Always test in a safe environment** with adequate space
2. **Keep emergency stop ready**: Know where the ESC key is
3. **Monitor robot status**: Watch the power and battery indicators
4. **Start with small movements**: Test each control before making large movements
5. **Unstow arm carefully**: Ensure adequate clearance before deploying arm

---

## Understanding the Code

### Architecture Overview

The WASD interface is built using a modular architecture with clear separation of concerns:

```
WasdInterface
├── ROS 2 Integration (publishers, subscribers, service clients)
├── Control Mode Management (base vs arm)
├── Command Processing (keyboard input handling)
├── Safety Systems (emergency stop, arm locking)
└── UI Management (curses-based display)
```

### Core Components

#### 1. Class Structure

**`WasdInterface`** - Main controller class
- Manages robot connection and control
- Handles user input and command routing
- Provides safety mechanisms

**`ExitCheck`** - Signal handling utility
- Captures SIGTERM/SIGINT for graceful shutdown
- Provides clean exit mechanism

**`SimpleSpotCommander`** - Robot command wrapper
- Abstracts basic robot operations
- Provides simplified interface for common commands

### Key Design Patterns

#### 1. Command Pattern
```python
self._base_command_dictionary = {
    ord("w"): self._move_forward,
    ord("s"): self._move_backward,
    # ... more commands
}
```
Each key maps to a specific command function, allowing easy extension and modification of controls.

#### 2. State Management
```python
self.control_mode = "base"  # or "arm"
self.is_arm_unstowed = False
self.is_arm_locked_in_position = False
```
Tracks robot and interface state to ensure safe operation.

#### 3. Publisher-Subscriber Pattern
```python
# Publishers for sending commands
self.pub_cmd_vel = self.node.create_publisher(Twist, ...)
self.pub_arm_vel = self.node.create_publisher(ArmVelocityCommandRequest, ...)

# Subscribers for status updates
self.sub_status_power_state = self.node.create_subscription(PowerState, ...)
self.sub_battery_state = self.node.create_subscription(BatteryStateArray, ...)
```

### Control Mechanisms

#### 1. Base Movement Control

**Velocity Control**:
- Uses `geometry_msgs/Twist` messages
- Continuous velocity commands with timeout
- Linear and angular velocity components

```python
def _velocity_cmd_helper(self, desc: str = "", v_x: float = 0.0, v_y: float = 0.0, v_rot: float = 0.0):
    twist = Twist()
    twist.linear.x = v_x
    twist.linear.y = v_y
    twist.angular.z = v_rot
    # Publish continuously for duration
    start_time = time.time()
    while time.time() - start_time < VELOCITY_CMD_DURATION:
        self.pub_cmd_vel.publish(twist)
        time.sleep(0.01)
```

#### 2. Arm Movement Control

**6-DOF Arm Control**:
- Uses cylindrical coordinate system (r, theta, z)
- End-effector rotation control
- Velocity-based control with acceleration limits

```python
def arm_velocity_cmd_helper(self, r=0.0, theta=0.0, z=0.0, 
                           end_effector_x=0.0, end_effector_y=0.0, end_effector_z=0.0):
    arm_cmd = ArmVelocityCommandRequest()
    # Set cylindrical velocities
    arm_cmd.command.cylindrical_velocity.linear_velocity.r = r
    arm_cmd.command.cylindrical_velocity.linear_velocity.theta = theta
    arm_cmd.command.cylindrical_velocity.linear_velocity.z = z
    # Set end-effector angular velocities
    arm_cmd.angular_velocity_of_hand_rt_odom_in_hand.x = end_effector_x
    # ... publish with duration control
```

### Safety Systems

#### 1. Emergency Stop
```python
def _stop(self) -> None:
    self.cli_stop.call_async(Trigger.Request())
```
Immediately stops all robot movement.

#### 2. Arm Position Locking
```python
def _velocity_cmd_helper(self, ...):
    if(self.is_arm_unstowed):
        if(not self.is_arm_locked_in_position):
            result = self.lock_arm_in_position()
            # Lock arm before base movement
```
When arm is deployed, it's locked in position before allowing base movement.

#### 3. Service Call Protection
```python
if(self.service_call_in_progress):
    self.add_message("Service call in progress, cannot send velocity command")
    return
```
Prevents conflicting commands during ongoing operations.

#### 4. QoS Configuration
```python
JOY_TELEOP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
    lifespan=rclpy.duration.Duration(seconds=1.0),
)
```
Optimized for real-time teleop control with low latency.

#### Service Clients
The interface uses multiple ROS 2 service clients for robot operations:
- Power control (`power_on`, `power_off`)
- Posture control (`sit`, `stand`, `self_right`)
- Arm control (`arm_stow`, `arm_unstow`)
- Gripper control (`open_gripper`, `close_gripper`)

### User Interface (Curses)

#### Display Management
```python
def _drive_draw(self, stdscr: curses.window) -> None:
    stdscr.clear()
    stdscr.addstr(0, 0, f"robot name: {self.robot_name}")
    stdscr.addstr(1, 0, f"CONTROL MODE: {self.control_mode.upper()}")
    # ... display status and help text
```

#### Input Processing
```python
def _drive_cmd(self, key: int) -> None:
    if self.control_mode == "base":
        cmd_function = self._base_command_dictionary[key]
    else:
        cmd_function = self._arm_command_dictionary[key]
    cmd_function()
```

### Configuration Constants

#### Movement Parameters
```python
VELOCITY_BASE_SPEED = 0.3       # m/s - base linear velocity
VELOCITY_BASE_ANGULAR = 0.5     # rad/sec - base angular velocity
ARM_VELOCITY_NORMALIZED = 0.4   # m/s - arm linear velocity
ARM_VELOCITY_ANGULAR_NORMALIZED = 0.3  # rad/s - arm angular velocity
```

#### Timing Parameters
```python
VELOCITY_CMD_DURATION = 1.0     # Duration to send base velocity commands
ARM_VELOCITY_CMD_DURATION = 0.25 # Duration to send arm velocity commands
COMMAND_INPUT_RATE = 0.1        # Rate limiting for input processing
```

#### Status Monitoring
The interface continuously monitors:
- **Battery status**: Charge levels for all battery packs
- **Power state**: Motor power on/off status
- **Arm state**: Stowed/unstowed status
- **Service call status**: Prevents command conflicts

### Extension Points

#### Adding New Commands
1. Add key mapping to appropriate command dictionary
2. Implement command function
3. Update help text in `_drive_draw()`

#### Adding New Control Modes
1. Extend `control_mode` state management
2. Create new command dictionary
3. Add mode switching logic
4. Update UI display

#### Customizing Movement Parameters
Modify the constants at the top of the file to adjust:
- Movement speeds
- Command durations
- Acceleration limits
- Input rates

This architecture provides a robust, extensible foundation for robot teleoperation while maintaining safety and usability.