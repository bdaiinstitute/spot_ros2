import argparse
from contextlib import closing
from typing import Optional

import synchros2.process as ros_process
from geometry_msgs.msg import Pose
from synchros2.subscription import Subscription

from .robot_commander import RobotCommander


class SpotRobotInterface:
    def __init__(self, robot_name: Optional[str] = None):
        # self.robot_name = os.getenv("ROBOT_NAME")
        self.robot_commander = RobotCommander(robot_name)
        self.latest_message = None  # Buffer to store the latest message
        self.is_busy = False  # Flag to indicate if the robot is busy
        self.robot_commander._logger.info(f"Robot Name: {self.robot_commander._robot_name}")

    def initialize(self) -> bool:
        if not self.robot_commander.initialize_robot():
            self.robot_commander._logger.info("Failed to initialize robot")
            return False
        self.robot_commander._logger.info("Initialized robot")
        return True

    def process_message(self, message: Pose) -> None:
        if self.is_busy:
            self.latest_message = message
            self.robot_commander._logger.info("Robot is busy, buffering the latest message")
        else:
            self.execute_command(message)

    def execute_command(self, message: Pose) -> None:
        self.is_busy = True
        self.robot_commander.walk_forward_with_vision_frame_goal(message)
        self.is_busy = False

    def listen_to_pose_commands(self) -> None:
        topic_data = Subscription(Pose, "/pose_commands")
        with closing(topic_data.stream()) as stream:
            for message in stream:
                self.process_message(message)


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", help="Name of the robot if the ROS driver is inside that namespace")
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    robot_interface = SpotRobotInterface(args.robot)
    if robot_interface.initialize():
        robot_interface.listen_to_pose_commands()


if __name__ == "__main__":
    main()
