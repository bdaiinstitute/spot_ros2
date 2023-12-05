#!/usr/bin/python3

# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import argparse
from typing import Optional

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
from bdai_ros2_wrappers.utilities import namespace_with
from utilities.tf_listener_wrapper import TFListenerWrapper
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME
from spot_utilities.spot_basic import SpotBasic

from spot_msgs.srv import Dock
from std_srvs.srv import Trigger

from spot_msgs.srv import GetInverseKinematicSolutions


def kinematic_request() -> GetInverseKinematicSolutions.Request:
    request = GetInverseKinematicSolutions.Request()
    request.request.header.client_name = "kinematic_client"
    request.request.header_is_set = True
    return request


def send_requests(robot_name: str, poses: int) -> bool:

    # Set up basic ROS2 utilities for communicating with the driver.
    node = ros_scope.node()
    if node is None:
        raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
    logger = node.get_logger()

    odom_frame_name = namespace_with(robot_name, ODOM_FRAME_NAME)
    grav_aligned_body_frame_name = namespace_with(robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)
    tf_listener = TFListenerWrapper(node)
    tf_listener.wait_for_a_tform_b(odom_frame_name, grav_aligned_body_frame_name)

    robot = SpotBasic(robot_name)

    # Claim robot.
    logger.info("Claiming robot")
    result = robot.claim()
    if not result:
        node.get_logger().error("Unable to claim robot")
        return False
    logger.info("Claimed robot")

    # Power on robot.
    logger.info("Powering robot on")
    result = robot.power_on()
    if not result:
        logger.error("Unable to power on robot")
        return False

    # Stand up robot.
    logger.info("Standing robot up")
    result = robot.stand()
    if not result:
        logger.error("Robot did not stand")
        return False
    logger.info("Successfully stood up.")

    # Send inverse kinematic requests.
    kinematics_client = node.create_client(GetInverseKinematicSolutions, "get_inverse_kinematics_solutions")
    if not kinematics_client.wait_for_service():
        logger.info("Service get_inverse_kinematics_solutions not available.")
        return False
    request = kinematic_request()
    response = kinematics_client.call(request)

    # Power off robot.
    logger.info("Powering robot off")
    result = robot.power_off()
    if not result:
        logger.error("Unable to power off robot")
        return False

    return True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, required=True, help="The robot name.")
    parser.add_argument("-n", "--poses", type=int, default=50, help="Number of desired tool poses to query.")
    args = parser.parse_args()
    send_requests(args.robot, args.poses)


if __name__ == "__main__":
    main()
