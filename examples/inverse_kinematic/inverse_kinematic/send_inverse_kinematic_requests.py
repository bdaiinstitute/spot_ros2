import argparse
from typing import Optional

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
from bdai_ros2_wrappers.utilities import namespace_with
from utilities.tf_listener_wrapper import TFListenerWrapper
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME
from utilities.simple_spot_commander import SimpleSpotCommander


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


def send_requests(robot_name: Optional[str] = None) -> bool:
    # Set up basic ROS2 utilities for communicating with the driver
    node = ros_scope.node()
    if node is None:
        raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
    logger = node.get_logger()

    odom_frame_name = namespace_with(robot_name, ODOM_FRAME_NAME)
    grav_aligned_body_frame_name = namespace_with(robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)
    tf_listener = TFListenerWrapper(node)
    tf_listener.wait_for_a_tform_b(odom_frame_name, grav_aligned_body_frame_name)

    robot = SimpleSpotCommander(robot_name, node)

    # Claim robot
    logger.info("Claiming robot")
    result = robot.command("claim")
    if not result.success:
        node.get_logger().error("Unable to claim robot message was " + result.message)
        return False
    logger.info("Claimed robot")

    # Stand the robot up.
    logger.info("Powering robot on")
    result = robot.command("power_on")
    if not result.success:
        logger.error("Unable to power on robot message was " + result.message)
        return False
    logger.info("Standing robot up")
    result = robot.command("stand")
    if not result.success:
        logger.error("Robot did not stand message was " + result.message)
        return False
    logger.info("Successfully stood up.")


    

    # TODO: send an inverse kinematic request.

    return True


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    send_requests(args.robot)


if __name__ == "__main__":
    main()
