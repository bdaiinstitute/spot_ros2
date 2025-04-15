# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import argparse
import logging
from typing import Any, Dict, Optional

import rclpy
import synchros2.process as ros_process
import synchros2.scope as ros_scope
from rcl_interfaces.srv import GetParameters
from rclpy.client import Client
from rclpy.node import Node
from std_srvs.srv import Trigger
from synchros2.service import Serviced
from synchros2.utilities import fqn, namespace_with

TRIGGER_SERVICES = [
    "claim",
    "release",
    "stop",
    "self_right",
    "sit",
    "stand",
    "power_on",
    "power_off",
    "estop/hard",
    "estop/gentle",
    "estop/release",
]

TRIGGER_SERVICES_ARM = [
    "arm_stow",
    "arm_unstow",
    "arm_carry",
    "open_gripper",
    "close_gripper",
]


class SimpleSpotCommander:
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        self._logger = logging.getLogger(fqn(self.__class__))
        node = node or ros_scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use synchros2.process.main?)")

        # add TRIGGER_SERVICES_ARM if the robot has an arm
        get_spot_parameters: Serviced[GetParameters.Request, GetParameters.Response] = Serviced(
            GetParameters, namespace_with(robot_name, "spot_ros2/get_parameters"), node=node
        )

        if not get_spot_parameters.wait_for_service(timeout_sec=5.0):
            self._logger.error(f"No {robot_name} driver found, assuming there is no arm")
            use_arm = False

        else:
            response = get_spot_parameters(GetParameters.Request(names=["has_arm"]), timeout_sec=5.0)
            param = response.values[0]
            use_arm = param.bool_value
            if use_arm:
                self._logger.info("Arm is available, trigger services of the arm will be available")
                TRIGGER_SERVICES.extend(TRIGGER_SERVICES_ARM)
            else:
                self._logger.info("Trigger services of the arm will not be available")

        self._command_map: Dict[str, Client] = {}
        for service_basename in TRIGGER_SERVICES:
            service_name = namespace_with(robot_name, service_basename)
            self._command_map[service_basename] = node.create_client(Trigger, service_name)
            self._logger.info(f"Waiting for service {service_basename}")
            self._command_map[service_basename].wait_for_service()
            self._logger.info(f"Found service {service_basename}")

    def command(self, command: str) -> Any:
        try:
            return self._command_map[command].call(Trigger.Request())
        except KeyError:
            err = f"No command {command}"
            self._logger.error(err)
            return Trigger.Response(success=False, message=err)


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("robot", help="Name of the robot if the ROS driver is inside that namespace")
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    commander = SimpleSpotCommander(args.robot)

    while rclpy.ok():
        cmd = input("Please enter a command:\n" + " ".join(TRIGGER_SERVICES) + "\n> ")
        result = commander.command(cmd)
        if not result.success:
            print("Error was", result.message)
        else:
            print("Successfully executed command")


if __name__ == "__main__":
    main()
