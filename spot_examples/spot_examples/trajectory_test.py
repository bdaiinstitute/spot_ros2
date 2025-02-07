import argparse
import logging
from typing import Optional

import synchros2.process as ros_process
import synchros2.scope as ros_scope
from rclpy.node import Node
from synchros2.action_client import ActionClientWrapper
from synchros2.utilities import fqn, namespace_with

from spot_msgs.action import Trajectory  # type: ignore

# from builtin_interfaces import Duration
from .simple_spot_commander import SimpleSpotCommander

# Where we want the robot to walk to relative to itself
# ROBOT_T_GOAL = SE2Pose(1.0, 0.0, 0.0)


class TrajectoryTest:
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        self._logger = logging.getLogger(fqn(self.__class__))
        node = node or ros_scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
        self._robot_name = robot_name

        # self._body_frame_name = namespace_with(self._robot_name, BODY_FRAME_NAME)
        # self._vision_frame_name = namespace_with(self._robot_name, VISION_FRAME_NAME)
        # self._tf_listener = TFListenerWrapper(node)
        # self._tf_listener.wait_for_a_tform_b(self._body_frame_name, self._vision_frame_name)
        self._robot = SimpleSpotCommander(self._robot_name, node)
        self._robot_command_client = ActionClientWrapper(
            Trajectory, namespace_with(self._robot_name, "trajectory"), node
        )

    def initialize_robot(self) -> bool:
        self._logger.info(f"Robot name: {self._robot_name}")
        self._logger.info("Claiming robot")
        result = self._robot.command("claim")
        if not result.success:
            self._logger.error("Unable to claim robot message was " + result.message)
            return False
        self._logger.info("Claimed robot")

        # Stand the robot up.
        self._logger.info("Powering robot on")
        result = self._robot.command("power_on")
        if not result.success:
            self._logger.error("Unable to power on robot message was " + result.message)
            return False
        self._logger.info("Standing robot up")
        result = self._robot.command("stand")
        if not result.success:
            self._logger.error("Robot did not stand message was " + result.message)
            return False
        self._logger.info("Successfully stood up.")
        return True

    def execute_trajectory(self) -> None:
        self._logger.info("Initiating trajectory")
        # geometry_msgs/PoseStamped target_pose
        # # After this duration, the command will time out and the robot will stop. Must be non-zero
        # builtin_interfaces/Duration duration
        # # If true, the feedback from the trajectory command must indicate that the robot is
        # # at the goal position. If set to false, the robot being near the goal is equivalent to
        # # it being at the goal. This is based on the feedback received from the boston dynamics
        # # API call at
        # # https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference.html?highlight=status_near_goal#se2trajectorycommand-feedback-status
        # bool precise_positioning
        # ---
        # bool success
        # string message
        # ---
        # string feedback

        traj = Trajectory.Goal()

        traj.target_pose.header.frame_id = "body"
        traj.duration.sec = 10
        traj.precise_positioning = True

        traj.target_pose.pose.position.x = 5.0
        traj.target_pose.pose.position.y = 0.0
        traj.target_pose.pose.position.z = 0.0
        traj.target_pose.pose.orientation.x = 0.0
        traj.target_pose.pose.orientation.y = 0.0
        traj.target_pose.pose.orientation.z = 0.0
        traj.target_pose.pose.orientation.w = 1.0

        self._robot_command_client.send_goal_async_handle(
            "trajectory", traj, feedback_callback=self._traj_feedback_callback
        )

    # def send_goal_async_handle(
    #     self,
    #     action_name: str,
    #     goal: Any,
    #     *,
    #     result_callback: Optional[Callable[[Any], None]] = None,
    #     feedback_callback: Optional[Callable[[Any], None]] = None,
    #     on_failure_callback: Optional[Callable[[], None]] = None,
    # ) -> ActionHandle:

    def _traj_feedback_callback(self, feedback: Trajectory.Feedback) -> None:
        self._logger.info(f"Trajectory feedback: {feedback.feedback}")


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


# rclpy.init(args=None)


@ros_process.main(cli(), autospin=True)
def main(args: argparse.Namespace) -> int:
    # node = rclpy.Node('trajectory_test_node')
    # node = main.node
    goto = TrajectoryTest(args.robot)
    goto.initialize_robot()
    goto.execute_trajectory()
    ros_process.wait_for_shutdown()
    # rclpy.spin(node)
    # rclpy.shutdown()
    return 0


if __name__ == "__main__":
    exit(main())
