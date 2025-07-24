#!/usr/bin/env python
# Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.
import argparse

import rclpy
from controller_manager_msgs.srv import (
    ConfigureController,
    LoadController,
    SetHardwareComponentState,
    SwitchController,
    UnloadController,
)
from lifecycle_msgs.msg import State
from rclpy.node import Node

from spot_msgs.msg import JointCommand  # type: ignore

# maximum and minimum joint angles in radians.
GRIPPER_OPEN_ANGLE = -1.57
GRIPPER_CLOSE_ANGLE = 0.0
GRIPPER_JOINT_NAME = "arm_f1x"

SPOT_HARDWARE_INTERFACE = "SpotSystem"


class SwitchState(Node):
    def __init__(self, robot_name: str | None) -> None:
        super().__init__("switch_state")
        self._robot_name = robot_name
        prefix = robot_name + "/" if robot_name is not None else ""
        self.get_logger().info(f"Robot name: {robot_name} {prefix=}")

        self._load_controller = self.create_client(LoadController, f"{prefix}controller_manager/load_controller")
        self._unload_controller = self.create_client(UnloadController, f"{prefix}controller_manager/unload_controller")
        self._configure_controller = self.create_client(
            ConfigureController, f"{prefix}controller_manager/configure_controller"
        )
        self._switch_controllers = self.create_client(SwitchController, f"{prefix}controller_manager/switch_controller")
        self._set_hardware_interface_state = self.create_client(
            SetHardwareComponentState, f"{prefix}controller_manager/set_hardware_component_state"
        )

        self._command_pub = self.create_publisher(JointCommand, f"{prefix}spot_joint_controller/joint_commands", 10)

    def connect(self, controller_name: str) -> None:
        print(f"Connecting to {controller_name}...")
        # activate hardware interface
        req = SetHardwareComponentState.Request()
        req.name = SPOT_HARDWARE_INTERFACE
        req.target_state.id = State.PRIMARY_STATE_ACTIVE
        future = self._set_hardware_interface_state.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print(f"Set hardware interface to active: {future.result()}")

        # load controller
        req = LoadController.Request()
        req.name = controller_name
        future = self._load_controller.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print(f"Load controller: {future.result()}")

        # configure controller
        req = ConfigureController.Request()
        req.name = controller_name
        future = self._configure_controller.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print(f"Configure controller: {future.result()}")

        # activate controller
        req = SwitchController.Request()
        req.activate_controllers = [controller_name]
        req.strictness = SwitchController.Request.STRICT
        future = self._switch_controllers.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print(f"Activate controller: {future.result()}")

    def disconnect(self, controller_name: str) -> None:
        print(f"Disconnecting from {controller_name}...")
        req = SwitchController.Request()
        req.deactivate_controllers = [controller_name]
        req.strictness = SwitchController.Request.STRICT
        future = self._switch_controllers.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print(f"Deactivate controller: {future.result()}")

        req = UnloadController.Request()
        req.name = controller_name
        future = self._unload_controller.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print(f"Unload controller: {future.result()}")

        req = SetHardwareComponentState.Request()
        req.name = SPOT_HARDWARE_INTERFACE
        req.target_state.id = State.PRIMARY_STATE_UNCONFIGURED
        future = self._set_hardware_interface_state.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print(f"Unconfigure hardware interface: {future.result()}")


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, help="Namespace the driver is in", default=None)
    parser_args = parser.parse_args()

    rclpy.init(args=args)

    switch_state = SwitchState(parser_args.robot)

    switch_state.connect("forward_position_controller")

    switch_state.disconnect("forward_position_controller")

    switch_state.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
