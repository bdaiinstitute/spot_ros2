"""WASD driving of robot with arm control mode."""
import argparse
import curses
import os
import signal
import sys
import threading
import time
from types import FrameType
from typing import Literal, Optional, Type

import rclpy
import synchros2.process as ros_process
import synchros2.scope as ros_scope
from bosdyn.client import ResponseError, RpcError
from bosdyn.api import geometry_pb2, robot_command_pb2
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from bosdyn_msgs.msg import ArmVelocityCommandRequest
from bosdyn_msgs.conversions import convert
from synchros2.action_client import ActionClientWrapper
from synchros2.utilities import namespace_with

from spot_msgs.action import RobotCommand  # type: ignore
from spot_msgs.msg import BatteryState, BatteryStateArray, PowerState  # type: ignore

from .simple_spot_commander import SimpleSpotCommander
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Recommended QoS for Joy/Teleop commands
JOY_TELEOP_QOS = QoSProfile(
    # Best Effort - prioritizes low latency over guaranteed delivery
    # Perfect for real-time control where latest command matters most
    reliability=ReliabilityPolicy.BEST_EFFORT,
    
    # Keep Last with small depth - only care about most recent commands
    # History of 1-5 is typical, 1 is often sufficient for teleop
    history=HistoryPolicy.KEEP_LAST,
    depth=1,  # Only keep the latest command
    
    # Volatile - don't persist commands after node restart
    # Teleop commands shouldn't be replayed from before restart
    durability=DurabilityPolicy.VOLATILE,

    # Short lifespan - commands expire quickly to avoid stale commands
    lifespan=rclpy.duration.Duration(seconds=1.0),  # Commands expire quickly
)

# -----------------------
# Constants
# -----------------------
VELOCITY_BASE_SPEED = 0.3       # m/s
VELOCITY_BASE_ANGULAR = 0.5     # rad/sec
VELOCITY_CMD_DURATION = 1.0     # seconds
COMMAND_INPUT_RATE = 0.1

ARM_MAXIMUM_ACCELERATION = 0.3  # m/s^2
ARM_MAX_LINEAR_VELOCITY = 0.5   # m/s
ARM_VELOCITY_CMD_DURATION = 0.25 # seconds

# Arm velocity increments
ARM_VELOCITY_NORMALIZED = 0.4 # m/s
ARM_VELOCITY_ANGULAR_NORMALIZED = 0.3  # rad/s


class ExitCheck(object):
    """A class to help exiting a loop, also capturing SIGTERM to exit the loop."""

    def __init__(self) -> None:
        self._kill_now = False
        signal.signal(signal.SIGTERM, self._sigterm_handler)
        signal.signal(signal.SIGINT, self._sigterm_handler)

    def __enter__(self) -> "ExitCheck":
        return self

    def __exit__(
        self,
        _type: Optional[Type[BaseException]],
        _value: Optional[BaseException],
        _traceback: Optional[FrameType],
    ) -> Literal[False]:
        return False

    def _sigterm_handler(self, _signum: int, _frame: Optional[FrameType]) -> None:
        self._kill_now = True

    def request_exit(self) -> None:
        """Manually trigger an exit (rather than sigterm/sigint)."""
        self._kill_now = True

    @property
    def kill_now(self) -> bool:
        """Return the status of the exit checker indicating if it should exit."""
        return self._kill_now


class WasdInterface:
    """A curses interface for driving the robot."""

    def __init__(self, robot_name: Optional[str] = None) -> None:
        self.robot_name = robot_name

        self.node = ros_scope.node()
        if self.node is None:
            raise ValueError("no ROS 2 node available (did you use synchros2.process.main?)")

        self.logger = self.node.get_logger()

        self._lock = threading.Lock()
        
        # Control mode: "base" or "arm"
        self.control_mode = "base"
        
        # Base movement commands
        self._base_command_dictionary = {
            27: self._stop,  # ESC key
            ord("\t"): self._quit_program,
            ord("r"): self._self_right,
            ord("p"): self._toggle_power,
            ord("v"): self._sit,
            ord("b"): self._battery_change_pose,
            ord("f"): self._stand,
            ord("w"): self._move_forward,
            ord("s"): self._move_backward,
            ord("a"): self._strafe_left,
            ord("d"): self._strafe_right,
            ord("q"): self._turn_left,
            ord("e"): self._turn_right,
            ord("y"): self._arm_unstow,
            ord("h"): self._arm_stow,
            ord("n"): self._open_gripper,
            ord("m"): self._close_gripper,
            ord("t"): self._toggle_control_mode,
            ord("T"): self._toggle_control_mode,
        }
        
        # Arm control commands (6 DOF)
        self._arm_command_dictionary = {
            27: self._stop,  # ESC key
            ord("\t"): self._quit_program,
            ord("r"): self._self_right,
            ord("p"): self._toggle_power,
            ord("v"): self._sit,
            ord("b"): self._battery_change_pose,
            ord("f"): self._stand,  # Keep same as base mode
            ord("w"): self._arm_move_out,      # Move arm forward/out
            ord("s"): self._arm_move_in,       # Move arm backward/in
            ord("a"): self._arm_rotate_ccw,    # Rotate arm counter-clockwise
            ord("d"): self._arm_rotate_cw,     # Rotate arm clockwise
            ord("q"): self._arm_move_up,       # Move arm up (changed from r)
            ord("e"): self._arm_move_down,     # Move arm down (changed from f)
            ord("i"): self._arm_rotate_plus_ry,  # End effector rotation
            ord("k"): self._arm_rotate_minus_ry,
            ord("u"): self._arm_rotate_plus_rx,
            ord("o"): self._arm_rotate_minus_rx,
            ord("j"): self._arm_rotate_plus_rz,
            ord("l"): self._arm_rotate_minus_rz,
            ord("y"): self._arm_unstow,
            ord("h"): self._arm_stow,
            ord("n"): self._open_gripper,
            ord("m"): self._close_gripper,
            ord("t"): self._toggle_control_mode,
            ord("T"): self._toggle_control_mode,
        }
        
        self._locked_messages = ["", "", ""]  # string: displayed message for user
        self._exit_check: Optional[ExitCheck] = None

        # Stuff that is set in start()
        self._robot_id = None

        self.pub_cmd_vel = self.node.create_publisher(Twist, namespace_with(robot_name, "cmd_vel"), JOY_TELEOP_QOS)
        self.pub_arm_vel = self.node.create_publisher(ArmVelocityCommandRequest, namespace_with(robot_name, "arm_velocity_commands"), JOY_TELEOP_QOS)

        self.latest_power_state_status: Optional[PowerState] = None
        self.latest_battery_status: Optional[BatteryStateArray] = None

        self.sub_status_power_state = self.node.create_subscription(
            PowerState, namespace_with(robot_name, "status/power_states"), self._status_power_state_callback, 1
        )
        self.sub_battery_state = self.node.create_subscription(
            BatteryStateArray, namespace_with(robot_name, "status/battery_states"), self._status_battery_callback, 1
        )

        self.cli_self_right = self.node.create_client(Trigger, namespace_with(robot_name, "self_right"))
        self.cli_sit = self.node.create_client(Trigger, namespace_with(robot_name, "sit"))
        self.cli_stand = self.node.create_client(Trigger, namespace_with(robot_name, "stand"))
        self.cli_stop = self.node.create_client(Trigger, namespace_with(robot_name, "stop"))
        self.cli_stow = self.node.create_client(Trigger, namespace_with(robot_name, "arm_stow"))
        self.cli_unstow = self.node.create_client(Trigger, namespace_with(robot_name, "arm_unstow"))
        self.cli_open_gripper = self.node.create_client(Trigger, namespace_with(robot_name, "open_gripper"))
        self.cli_close_gripper = self.node.create_client(Trigger, namespace_with(robot_name, "close_gripper"))
        self.cli_power_off = self.node.create_client(Trigger, namespace_with(robot_name, "power_off"))
        self.cli_power_on = self.node.create_client(Trigger, namespace_with(robot_name, "power_on"))
        self.cli_rollover = self.node.create_client(Trigger, namespace_with(robot_name, "rollover"))

        self.robot = SimpleSpotCommander(robot_name, self.node)
        self.robot_command_client = ActionClientWrapper(
            RobotCommand, namespace_with(robot_name, "robot_command"), self.node
        )

        self.is_arm_unstowed = False
        self.is_arm_locked_in_position = False
        self.service_call_in_progress = False

    def start(self) -> bool:
        # """Begin communication with the robot."""

        # Claim robot
        self.logger.info("Claiming robot")
        result = self.robot.command("claim")
        if not result.success:
            self.logger.error("Unable to claim robot message was " + result.message)
            return False
        self.logger.info("Claimed robot")

        # Power on robot
        self.logger.info("Powering on robot")
        result = self.robot.command("power_on")
        if not result.success:
            self.logger.error("Unable to power on robot message was " + result.message)
            return False

        # self.logger.info("Standing robot up")
        # result = self.robot.command("stand")
        # if not result.success:
        #     self.logger.error("Robot did not stand message was " + result.message)
        #     return False
        self.logger.info("Successfully powered on the robot.")
        time.sleep(3)

        return True

    def flush_buffer(self, stdscr: curses.window) -> None:
        """Manually flush the curses input buffer"""
        key = 0
        while key != -1:
            key = stdscr.getch()

    def add_message(self, msg_text: str) -> None:
        """Display the given message string to the user in the curses interface."""
        with self._lock:
            self._locked_messages = [msg_text] + self._locked_messages[:-1]

    def message(self, idx: int) -> str:
        """Grab one of the 3 last messages added."""
        with self._lock:
            return self._locked_messages[idx]

    def _status_power_state_callback(self, msg: PowerState) -> None:
        self.latest_power_state_status = msg

    def _status_battery_callback(self, msg: BatteryState) -> None:
        self.latest_battery_status = msg

    def _toggle_control_mode(self) -> None:
        """Toggle between base movement and arm control modes."""
        if self.control_mode == "base":
            self.control_mode = "arm"
            self.add_message("Switched to ARM CONTROL mode")
        else:
            self.control_mode = "base"
            self.add_message("Switched to BASE MOVEMENT mode")

    def drive(self, stdscr: curses.window) -> None:
        """User interface to control the robot via the passed-in curses screen interface object."""
        with ExitCheck() as self._exit_check:
            stdscr.nodelay(True)  # Don't block for user input.
            stdscr.resize(30, 140)  # Increased height for more info
            stdscr.refresh()

            # for debug
            curses.echo()

            self.logger.info("Starting WASD interface")

            # try:
            while not self._exit_check.kill_now:
                self._drive_draw(stdscr)

                try:
                    cmd = stdscr.getch()
                    # Do not queue up commands on client
                    self.flush_buffer(stdscr)
                    self._drive_cmd(cmd)
                    time.sleep(COMMAND_INPUT_RATE)
                except Exception:
                    # On robot command fault, sit down safely before killing the program.
                    if self.is_arm_unstowed:
                        self.cli_stow.call_async(Trigger.Request())
                    self.cli_sit.call_async(Trigger.Request())
                    self.cli_power_off.call_async(Trigger.Request())
                    time.sleep(2.0)
                    raise

    # -----------------------
    # Draw update
    # -----------------------
    def _drive_draw(self, stdscr: curses.window) -> None:
        stdscr.clear()  # clear screen
        stdscr.resize(30, 140)
        
        # Basic info
        stdscr.addstr(0, 0, f"robot name: {self.robot_name}")
        stdscr.addstr(1, 0, f"CONTROL MODE: {self.control_mode.upper()}")
        stdscr.addstr(2, 0, self._battery_str())
        stdscr.addstr(3, 0, self._power_state_str())
        
        # Messages
        for i in range(3):
            stdscr.addstr(5 + i, 2, self.message(i))
        
        # Commands - show different help based on mode
        if self.control_mode == "base":
            stdscr.addstr(9, 0, "=== BASE MOVEMENT MODE ===")
            stdscr.addstr(10, 0, "commands: [tab]: quit, [T]: toggle to ARM mode    ")
            stdscr.addstr(11, 0, "          [p]: power, [f]: stand, [v]: sit        ")
            stdscr.addstr(12, 0, "          [r]: self-right, [b]: battery change    ")
            stdscr.addstr(13, 0, "          [wasd]: directional strafing            ")
            stdscr.addstr(14, 0, "          [qe]: turning, [ESC]: stop              ")
            stdscr.addstr(15, 0, "          [y]: unstow arm, [h]: stow arm          ")
            stdscr.addstr(16, 0, "          [n/m]: open/close gripper               ")
        else:
            stdscr.addstr(9, 0, "=== ARM CONTROL MODE ===")
            stdscr.addstr(10, 0, "commands: [tab]: quit, [T]: toggle to BASE mode   ")
            stdscr.addstr(11, 0, "          [p]: power, [f]: stand, [v]: sit        ")
            stdscr.addstr(12, 0, "          [r]: self-right, [b]: battery change    ")
            stdscr.addstr(13, 0, "          [ws]: arm forward/back                  ")
            stdscr.addstr(14, 0, "          [ad]: arm rotate CCW/CW                 ")
            stdscr.addstr(15, 0, "          [qe]: arm up/down                       ")
            stdscr.addstr(16, 0, "          [ik]: end effector Y-rot +/-            ")
            stdscr.addstr(17, 0, "          [uo]: end effector X-rot +/-            ")
            stdscr.addstr(18, 0, "          [jl]: end effector Z-rot +/-            ")
            stdscr.addstr(19, 0, "          [y]: unstow arm, [h]: stow arm          ")
            stdscr.addstr(20, 0, "          [n/m]: open/close gripper, [ESC]: stop ")
        
        stdscr.refresh()

    def _drive_cmd(self, key: int) -> None:
        """Run user commands at each update."""
        try:
            # Use the appropriate command dictionary based on current mode
            if self.control_mode == "base":
                cmd_function = self._base_command_dictionary[key]
            else:
                cmd_function = self._arm_command_dictionary[key]
            cmd_function()

        except KeyError:
            if key and key != -1 and key < 256:
                self.add_message(f"Unrecognized keyboard command in {self.control_mode} mode: '{chr(key)}'")

    def _battery_str(self) -> str:
        """Return the battery as a string."""
        if self.latest_battery_status is None:
            return "Battery: (unknown)"
        
        battery_states = []
        for battery in self.latest_battery_status.battery_states:
            battery_states.append(f"{battery.identifier}: {battery.charge_percentage:.1f}%")
        
        return "Battery: " + ", ".join(battery_states)

    def _power_state_str(self) -> str:
        """Return the power state as a string."""
        if self.latest_power_state_status is None:
            return "Power: (unknown)"
        
        state_map = {
            PowerState.STATE_UNKNOWN: "UNKNOWN",
            PowerState.STATE_OFF: "OFF", 
            PowerState.STATE_ON: "ON"
        }
        
        motor_power = state_map.get(self.latest_power_state_status.motor_power_state, "UNKNOWN")
        return f"Power: Motor={motor_power}"

    def _quit_program(self) -> None:
        self._sit()
        if self._exit_check is not None:
            self._exit_check.request_exit()

    def _toggle_power(self) -> None:
        power_state = self.latest_power_state_status
        if power_state is None:
            self.add_message("Could not toggle power because power state is unknown")
            return

        if power_state.motor_power_state == PowerState.STATE_OFF:
            self.cli_power_on.call_async(Trigger.Request())
        else:
            self.cli_power_off.call_async(Trigger.Request())

    def _self_right(self) -> None:
        self.cli_self_right.call_async(Trigger.Request())

    def _battery_change_pose(self) -> None:
        self.cli_rollover.call_async(Trigger.Request())

    def _sit(self) -> None:
        self.cli_sit.call_async(Trigger.Request())

    def _stand(self) -> None:
        self.cli_stand.call_async(Trigger.Request())
    
    def _arm_stow(self) -> None:
        self.cli_stow.call_async(Trigger.Request())
        self.is_arm_unstowed = False
    
    def _arm_unstow(self) -> None:
        self.cli_unstow.call_async(Trigger.Request())
        self.is_arm_unstowed = True
    
    def _open_gripper(self) -> None:
        self.cli_open_gripper.call_async(Trigger.Request())
    
    def _close_gripper(self) -> None:
        self.cli_close_gripper.call_async(Trigger.Request())

    # -----------------------
    # Base movement functions
    # -----------------------
    def _move_forward(self) -> None:
        self._velocity_cmd_helper("move_forward", v_x=VELOCITY_BASE_SPEED)

    def _move_backward(self) -> None:
        self._velocity_cmd_helper("move_backward", v_x=-VELOCITY_BASE_SPEED)

    def _strafe_left(self) -> None:
        self._velocity_cmd_helper("strafe_left", v_y=VELOCITY_BASE_SPEED)

    def _strafe_right(self) -> None:
        self._velocity_cmd_helper("strafe_right", v_y=-VELOCITY_BASE_SPEED)

    def _turn_left(self) -> None:
        self._velocity_cmd_helper("turn_left", v_rot=VELOCITY_BASE_ANGULAR)

    def _turn_right(self) -> None:
        self._velocity_cmd_helper("turn_right", v_rot=-VELOCITY_BASE_ANGULAR)

    def _stop(self) -> None:
        self.cli_stop.call_async(Trigger.Request())

    def _velocity_cmd_helper(self, desc: str = "", v_x: float = 0.0, v_y: float = 0.0, v_rot: float = 0.0) -> None:
        if(self.service_call_in_progress):
            self.add_message("Service call in progress, cannot send velocity command")
            return
        if(self.is_arm_unstowed):
            if(not self.is_arm_locked_in_position):
                self.service_call_in_progress = True
                result = self.lock_arm_in_position()
                self.service_call_in_progress = False
                if not result.success:
                    self.add_message("Failed to lock arm in position, cannot send velocity command")
                    self.is_arm_locked_in_position = False
                    return
                else:
                    self.is_arm_locked_in_position = True
        else:
            self.is_arm_locked_in_position = False
        twist = Twist()
        twist.linear.x = v_x
        twist.linear.y = v_y
        twist.angular.z = v_rot
        start_time = time.time()
        while time.time() - start_time < VELOCITY_CMD_DURATION:
            self.pub_cmd_vel.publish(twist)
            time.sleep(0.01)
        self.pub_cmd_vel.publish(Twist())

    # -----------------------
    # Arm movement functions (6 DOF)
    # -----------------------
    def _arm_move_out(self) -> None:
        self.arm_velocity_cmd_helper("arm_move_out", r=ARM_VELOCITY_NORMALIZED)
        self.add_message("Arm: move out")

    def _arm_move_in(self) -> None:
        self.arm_velocity_cmd_helper("arm_move_in", r=-ARM_VELOCITY_NORMALIZED)
        self.add_message("Arm: move in")

    def _arm_rotate_ccw(self) -> None:
        self.arm_velocity_cmd_helper("arm_rotate_ccw", theta=ARM_VELOCITY_NORMALIZED)
        self.add_message("Arm: rotate CCW")

    def _arm_rotate_cw(self) -> None:
        self.arm_velocity_cmd_helper("arm_rotate_cw", theta=-ARM_VELOCITY_NORMALIZED)
        self.add_message("Arm: rotate CW")

    def _arm_move_up(self) -> None:
        self.arm_velocity_cmd_helper("arm_move_up", z=ARM_VELOCITY_NORMALIZED)
        self.add_message("Arm: move up")

    def _arm_move_down(self) -> None:
        self.arm_velocity_cmd_helper("arm_move_down", z=-ARM_VELOCITY_NORMALIZED)
        self.add_message("Arm: move down")

    def _arm_rotate_plus_ry(self) -> None:
        self.arm_velocity_cmd_helper("arm_rotate_plus_ry", end_effector_y=ARM_VELOCITY_ANGULAR_NORMALIZED)
        self.add_message("End effector: +Y rotation")

    def _arm_rotate_minus_ry(self) -> None:
        self.arm_velocity_cmd_helper("arm_rotate_minus_ry", end_effector_y=-ARM_VELOCITY_ANGULAR_NORMALIZED)
        self.add_message("End effector: -Y rotation")

    def _arm_rotate_plus_rx(self) -> None:
        self.arm_velocity_cmd_helper("arm_rotate_plus_rx", end_effector_x=ARM_VELOCITY_ANGULAR_NORMALIZED)
        self.add_message("End effector: +X rotation")

    def _arm_rotate_minus_rx(self) -> None:
        self.arm_velocity_cmd_helper("arm_rotate_minus_rx", end_effector_x=-ARM_VELOCITY_ANGULAR_NORMALIZED)
        self.add_message("End effector: -X rotation")

    def _arm_rotate_plus_rz(self) -> None:
        self.arm_velocity_cmd_helper("arm_rotate_plus_rz", end_effector_z=ARM_VELOCITY_ANGULAR_NORMALIZED)
        self.add_message("End effector: +Z rotation")

    def _arm_rotate_minus_rz(self) -> None:
        self.arm_velocity_cmd_helper("arm_rotate_minus_rz", end_effector_z=-ARM_VELOCITY_ANGULAR_NORMALIZED)
        self.add_message("End effector: -Z rotation")

    def arm_velocity_cmd_helper(self, desc: str = "", 
                                r: float = 0.0, 
                                theta: float = 0.0, 
                                z: float = 0.0,
                                end_effector_x: float = 0.0,
                                end_effector_y: float = 0.0,
                                end_effector_z: float = 0.0,
                                ) -> None:
        
        arm_cmd = ArmVelocityCommandRequest()
        arm_cmd.angular_velocity_of_hand_rt_odom_in_hand.x = end_effector_x
        arm_cmd.angular_velocity_of_hand_rt_odom_in_hand.y = end_effector_y
        arm_cmd.angular_velocity_of_hand_rt_odom_in_hand.z = end_effector_z

        arm_cmd.maximum_acceleration.data = ARM_MAXIMUM_ACCELERATION
        arm_cmd.command.command_choice = arm_cmd.command.COMMAND_CYLINDRICAL_VELOCITY_SET
        arm_cmd.command.cylindrical_velocity.linear_velocity.r = r
        arm_cmd.command.cylindrical_velocity.linear_velocity.theta = theta
        arm_cmd.command.cylindrical_velocity.linear_velocity.z = z
        arm_cmd.command.cylindrical_velocity.max_linear_velocity.data = ARM_MAX_LINEAR_VELOCITY
        start_time = time.time()
        while time.time() - start_time < ARM_VELOCITY_CMD_DURATION:
            self.pub_arm_vel.publish(arm_cmd)
            time.sleep(0.01)
        self.pub_arm_vel.publish(ArmVelocityCommandRequest())

    def lock_arm_in_position(self) -> None:
        """Lock arm in current position - placeholder for actual implementation"""
        # This would need to be implemented based on your robot's API
        self.add_message("Locking arm in current position")
        robot_command = robot_command_pb2.RobotCommand()
        arm_command = robot_command.synchronized_command.arm_command
        arm_command.arm_joint_move_command.trajectory.points.add()
        action_goal = RobotCommand.Goal()
        convert(robot_command, action_goal.command)
        return self.robot_command_client.send_goal_and_wait("lock_arm_in_place", action_goal)


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> bool:
    """Command-line interface."""

    wasd_interface = WasdInterface(args.robot)
    try:
        wasd_interface.start()
    except (ResponseError, RpcError):
        wasd_interface.logger.error(
            "Failed to initialize robot communication: " + str(ResponseError) + " " + str(RpcError)
        )
        return False

    # try:
    os.environ.setdefault("ESCDELAY", "0")
    curses.wrapper(wasd_interface.drive)
    # except Exception as e:
    #     wasd_interface.logger.error("WASD has thrown an error:" + str(e))

    return True


if __name__ == "__main__":
    if not main():
        sys.exit(1)