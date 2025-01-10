# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import argparse
import logging
from typing import Any, Dict, Optional

import rclpy
import synchros2.process as ros_process
import synchros2.scope as ros_scope
from rclpy.client import Client
from rclpy.node import Node
from std_srvs.srv import Trigger
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


class SimpleSpotCommander:
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        self._logger = logging.getLogger(fqn(self.__class__))
        node = node or ros_scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
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
