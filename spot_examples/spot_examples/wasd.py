"""WASD driving of robot."""
import argparse
import curses
import os
import signal
import sys
import threading
import time
from types import FrameType
from typing import Literal, Optional, Type

import synchros2.process as ros_process
import synchros2.scope as ros_scope
from bosdyn.client import ResponseError, RpcError
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from synchros2.action_client import ActionClientWrapper
from synchros2.utilities import namespace_with

from spot_msgs.action import RobotCommand  # type: ignore
from spot_msgs.msg import BatteryState, BatteryStateArray, PowerState  # type: ignore

from .simple_spot_commander import SimpleSpotCommander

VELOCITY_BASE_SPEED = 0.3  # m/s
VELOCITY_BASE_ANGULAR = 0.5  # rad/sec
VELOCITY_CMD_DURATION = 1.0  # seconds
COMMAND_INPUT_RATE = 0.1


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
            raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")

        self.logger = self.node.get_logger()

        self._lock = threading.Lock()
        self._command_dictionary = {
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
            ord("u"): self._unstow,
            ord("j"): self._stow,
        }
        self._locked_messages = ["", "", ""]  # string: displayed message for user
        self._exit_check: Optional[ExitCheck] = None

        # Stuff that is set in start()
        self._robot_id = None

        self.pub_cmd_vel = self.node.create_publisher(Twist, namespace_with(robot_name, "cmd_vel"), 1)

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
        self.cli_power_off = self.node.create_client(Trigger, namespace_with(robot_name, "power_off"))
        self.cli_power_on = self.node.create_client(Trigger, namespace_with(robot_name, "power_on"))
        self.cli_rollover = self.node.create_client(Trigger, namespace_with(robot_name, "rollover"))

        self.robot = SimpleSpotCommander(robot_name, self.node)
        self.robot_command_client = ActionClientWrapper(
            RobotCommand, namespace_with(robot_name, "robot_command"), self.node
        )

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

        self.logger.info("Standing robot up")
        result = self.robot.command("stand")
        if not result.success:
            self.logger.error("Robot did not stand message was " + result.message)
            return False
        self.logger.info("Successfully stood up.")
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

    def drive(self, stdscr: curses.window) -> None:
        """User interface to control the robot via the passed-in curses screen interface object."""
        with ExitCheck() as self._exit_check:
            stdscr.nodelay(True)  # Don't block for user input.
            stdscr.resize(26, 140)
            stdscr.refresh()

            # for debug
            curses.echo()

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
                    self.cli_sit.call_async(Trigger.Request())
                    self.cli_power_off.call_async(Trigger.Request())
                    time.sleep(2.0)
                    raise

    def _drive_draw(self, stdscr: curses.window) -> None:
        """Draw the interface screen at each update."""
        stdscr.clear()  # clear screen
        stdscr.resize(26, 140)
        stdscr.addstr(0, 0, f"robot name: {self.robot_name:20s}")
        stdscr.addstr(2, 0, self._battery_str())
        stdscr.addstr(3, 0, self._power_state_str())
        for i in range(3):
            stdscr.addstr(7 + i, 2, self.message(i))
        stdscr.addstr(10, 0, "commands: [tab]: quit, [p]: power                   ")
        stdscr.addstr(11, 0, "          [f]: stand, [r]: self-right               ")
        stdscr.addstr(12, 0, "          [v]: sit, [b]: battery change             ")
        stdscr.addstr(13, 0, "          [wasd]: directional strafing              ")
        stdscr.addstr(14, 0, "          [qe]: turning, [ESC]: stop                ")
        stdscr.addstr(15, 0, "          [u]: unstow arm, [j]: stow arm            ")
        stdscr.addstr(16, 0, "")

        stdscr.refresh()

    def _drive_cmd(self, key: int) -> None:
        """Run user commands at each update."""
        try:
            cmd_function = self._command_dictionary[key]
            cmd_function()

        except KeyError:
            if key and key != -1 and key < 256:
                self.add_message(f"Unrecognized keyboard command: '{chr(key)}'")

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
        twist = Twist()
        twist.linear.x = v_x
        twist.linear.y = v_y
        twist.angular.z = v_rot
        start_time = time.time()
        while time.time() - start_time < VELOCITY_CMD_DURATION:
            self.pub_cmd_vel.publish(twist)
            time.sleep(0.01)
        self.pub_cmd_vel.publish(Twist())

    def _stow(self) -> None:
        self.cli_stow.call_async(Trigger.Request())

    def _unstow(self) -> None:
        self.cli_unstow.call_async(Trigger.Request())

    def _power_state_str(self) -> str:
        power_state = self.latest_power_state_status
        if power_state is None:
            return "latest_power_state_status is None"

        motor_power_state_str = {
            PowerState.STATE_UNKNOWN: "Unknown",
            PowerState.STATE_OFF: "Off",
            PowerState.STATE_ON: "On",
            PowerState.STATE_POWERING_ON: "Powering On",
            PowerState.STATE_POWERING_OFF: "Powering Off",
            PowerState.STATE_ERROR: "Error",
        }

        shore_power_state_str = {
            PowerState.STATE_UNKNOWN_SHORE_POWER: "Unknown Shore Power",
            PowerState.STATE_ON_SHORE_POWER: "On Shore Power",
            PowerState.STATE_OFF_SHORE_POWER: "Off Shore Power",
        }

        motor_state = motor_power_state_str.get(power_state.motor_power_state, "Invalid State")
        shore_state = shore_power_state_str.get(power_state.shore_power_state, "Invalid State")

        return f"motor power state: {motor_state}, shore power state: {shore_state}"

    def _battery_str(self) -> str:
        battery_state_array = self.latest_battery_status
        if not battery_state_array:
            return "latest_battery_status is None"
        battery_state = battery_state_array.battery_states[0]
        status_str = {
            BatteryState.STATUS_UNKNOWN: "Unknown",
            BatteryState.STATUS_MISSING: "Missing",
            BatteryState.STATUS_CHARGING: "Charging",
            BatteryState.STATUS_DISCHARGING: "Discharging",
            BatteryState.STATUS_BOOTING: "Booting",
        }

        status = status_str.get(battery_state.status, "Invalid Status")

        percent = battery_state.charge_percentage
        return f"battery: {percent}% ({status})"


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

    try:
        # try:
        # Prevent curses from introducing a 1-second delay for ESC key
        os.environ.setdefault("ESCDELAY", "0")
        # Run wasd interface in curses mode, then restore terminal config.
        curses.wrapper(wasd_interface.drive)
    except Exception as e:
        wasd_interface.logger.error("WASD has thrown an error:" + str(e))

    return True


if __name__ == "__main__":
    if not main():
        sys.exit(1)
