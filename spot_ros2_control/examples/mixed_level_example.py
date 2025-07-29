#!/usr/bin/env python
# Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.
import argparse
import time

import rclpy
import rclpy.client
from controller_manager_msgs.srv import (
    ConfigureController,
    LoadController,
    SetHardwareComponentState,
    SwitchController,
    UnloadController,
)
from lifecycle_msgs.msg import State
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from spot_msgs.msg import JointCommand  # type: ignore

# maximum and minimum joint angles in radians.
GRIPPER_OPEN_ANGLE = -1.57
GRIPPER_CLOSE_ANGLE = 0.0
GRIPPER_JOINT_NAME = "arm_f1x"

SPOT_HARDWARE_INTERFACE = "SpotSystem"


class SwitchState(Node):
    def __init__(self, robot_name: str | None) -> None:
        super().__init__("switch_state")
        self.prefix = robot_name + "/" if robot_name is not None else ""
        self.get_logger().info(f"Robot name: {robot_name}")

        # services for connecting and disconnecting from ROS 2 controllers
        self._load_controller = self.create_client(LoadController, self.prefix + "controller_manager/load_controller")
        self._unload_controller = self.create_client(
            UnloadController, self.prefix + "controller_manager/unload_controller"
        )
        self._configure_controller = self.create_client(
            ConfigureController, self.prefix + "controller_manager/configure_controller"
        )
        self._switch_controllers = self.create_client(
            SwitchController, self.prefix + "controller_manager/switch_controller"
        )
        self._set_hardware_interface_state = self.create_client(
            SetHardwareComponentState, self.prefix + "controller_manager/set_hardware_component_state"
        )

        # for low level command
        # Here we are only moving the gripper, similarly to set_gripper_gains.py as a minimal example
        self._command_pub = self.create_publisher(
            JointCommand, self.prefix + "spot_joint_controller/joint_commands", 10
        )
        self._joint_command = JointCommand()
        self._joint_command.name = [self.prefix + GRIPPER_JOINT_NAME]
        self.current_gripper_angle: float | None = None

        # services for the high level spot driver. Again a minimal list for a simple example.
        self._claim = self.create_client(Trigger, self.prefix + "claim")
        self._stand = self.create_client(Trigger, self.prefix + "stand")
        self._sit = self.create_client(Trigger, self.prefix + "sit")
        self._power_on = self.create_client(Trigger, self.prefix + "power_on")

    def connect(self, controller_names: list[str]) -> None:
        print(f"Connecting to {controller_names}...")
        # activate hardware interface
        req = SetHardwareComponentState.Request()
        req.name = SPOT_HARDWARE_INTERFACE
        req.target_state.id = State.PRIMARY_STATE_ACTIVE
        future = self._set_hardware_interface_state.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print(f"Set hardware interface to active: {future.result()}")

        for controller in controller_names:
            # load controller
            req = LoadController.Request()
            req.name = controller
            future = self._load_controller.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            print(f"Load {controller}: {future.result()}")

            # configure controller
            req = ConfigureController.Request()
            req.name = controller
            future = self._configure_controller.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            print(f"Configure {controller}: {future.result()}")

        # activate controller
        req = SwitchController.Request()
        req.activate_controllers = controller_names
        req.strictness = SwitchController.Request.STRICT
        future = self._switch_controllers.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print(f"Activate controller: {future.result()}")

    def disconnect(self, controller_names: list[str]) -> None:
        print(f"Disconnecting from {controller_names}...")
        # Deactivate the controllers
        req = SwitchController.Request()
        req.deactivate_controllers = controller_names
        req.strictness = SwitchController.Request.STRICT
        future = self._switch_controllers.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print(f"Deactivate controller: {future.result()}")

        for controller in controller_names:
            # unload each controller
            req = UnloadController.Request()
            req.name = controller
            future = self._unload_controller.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            print(f"Unload {controller}: {future.result()}")

        # unconfigure the hardware interface
        req = SetHardwareComponentState.Request()
        req.name = SPOT_HARDWARE_INTERFACE
        req.target_state.id = State.PRIMARY_STATE_UNCONFIGURED
        future = self._set_hardware_interface_state.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print(f"Unconfigure hardware interface: {future.result()}")

    def _trigger_wrapper(self, client: rclpy.client.Client, name: str) -> None:
        # This is a simple wrapper around the high level spot driver services
        self.get_logger().info(f"{name}")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Result: {future.result()}")

    def power_on(self) -> None:
        self._trigger_wrapper(self._power_on, "Powering on")

    def claim(self) -> None:
        self._trigger_wrapper(self._claim, "Claiming")

    def stand(self) -> None:
        self._trigger_wrapper(self._stand, "Standing")

    def sit(self) -> None:
        self._trigger_wrapper(self._sit, "Sitting")

    def move_gripper(self, goal_joint_angle: float) -> None:
        """Command the gripper to a given joint angle by streaming a command."""
        self._joint_command.position = [goal_joint_angle]
        self._command_pub.publish(self._joint_command)

    def _joint_state_callback(self, msg: JointState) -> None:
        gripper_index = msg.name.index(self.prefix + GRIPPER_JOINT_NAME)
        self.current_gripper_angle = msg.position[gripper_index]

    def _get_gripper_angle(self) -> float:
        self.current_gripper_angle = None
        # This grabs the gripper joint angle from the ROS 2 control joint state broadcaster
        gripper_sub = self.create_subscription(
            JointState, self.prefix + "low_level/joint_states", self._joint_state_callback, 10
        )
        while self.current_gripper_angle is None:
            rclpy.spin_once(self)
        self.get_logger().info(f"Gripper joint angle: {self.current_gripper_angle}")  # type: ignore
        self.destroy_subscription(gripper_sub)
        return self.current_gripper_angle

    def open_and_close(self, duration_sec: float = 1.0, frequency_hz: float = 50.0) -> None:
        """Open and close the gripper by streaming position commands.

        Args:
            duration_sec (float): Duration in seconds of each open and close movement
            frequency_hz (int): Frequency in Hz of the command publish rate.
        """
        # get current gripper angle from low level joint states topic
        starting_gripper_angle = self._get_gripper_angle()

        # most of this logic is taken from set_gripper_gains.py
        npoints = int(duration_sec * frequency_hz)
        dt = 1.0 / frequency_hz
        step_size_open = (GRIPPER_OPEN_ANGLE - starting_gripper_angle) / npoints
        self._logger.info("Opening...")
        for i in range(npoints):
            self.move_gripper(goal_joint_angle=(starting_gripper_angle + i * step_size_open))
            time.sleep(dt)
        self._logger.info("Closing...")
        step_size_close = (GRIPPER_OPEN_ANGLE - GRIPPER_CLOSE_ANGLE) / npoints
        for i in range(npoints):
            self.move_gripper(goal_joint_angle=(GRIPPER_OPEN_ANGLE - i * step_size_close))
            time.sleep(dt)


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, help="Namespace the driver is in", default=None)
    parser_args = parser.parse_args()

    rclpy.init(args=args)

    # in this example, we will activate and deactivate the spot joint controller for streaming joint commands
    # and the joint state broadcaster for getting the robot's joint states.
    controllers = ["spot_joint_controller", "joint_state_broadcaster"]

    switch_state = SwitchState(parser_args.robot)

    # we first start by interacting with the spot driver provided interface --
    # claiming the lease, powering the robot on, and sending a high level "stand" command.
    switch_state.claim()
    switch_state.power_on()
    switch_state.stand()

    # next we bring up the hardware interface and relevant controllers in the Spot ROS 2 control stack
    input("press to connect to joint controller")
    switch_state.connect(controllers)

    # here we interact with the spot joint controller and joint state broadcaster we activated to send commands
    input("press to open and close")
    switch_state.open_and_close()

    # next, bring down the hardware interface and controllers to prepare for "high level" commands again
    input("press to disconnect")
    switch_state.disconnect(controllers)

    # finally, we are back to where we started, sending high level commands to the spot driver!
    input("press to sit")
    switch_state.sit()

    switch_state.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
