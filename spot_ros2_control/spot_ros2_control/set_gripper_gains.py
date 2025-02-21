#!/usr/bin/env python
# Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.
import time

import synchros2.process as ros_process
from synchros2.node import Node

from spot_msgs.msg import JointCommand

# maximum and minimum joint angles in radians.
GRIPPER_OPEN_ANGLE = -1.57
GRIPPER_CLOSE_ANGLE = 0.0


class ExampleGripperStreaming:
    def __init__(self, node: Node) -> None:
        """Initialize the example."""
        self._node = node
        self._logger = self._node.get_logger()
        self._command_pub = self._node.create_publisher(JointCommand, "spot_forward_controller/joint_commands", 10)
        self._joint_command = JointCommand()
        self._joint_command.name = ["arm_f1x"]
        self._joint_command.k_q_p = [16.0]
        self._joint_command.k_qd_p = [0.32]

    def get_k_q_p(self):
        """Get the current k_q_p command for the gripper."""
        return self._joint_command.k_q_p[0]

    def get_k_qd_p(self):
        """Get the current k_qd_p command for the gripper."""
        return self._joint_command.k_qd_p[0]

    def set_gains(self, k_q_p: float, k_qd_p: float) -> None:
        """Set the k_q_p and k_qd_p commands for the gripper."""
        self._joint_command.k_q_p = [k_q_p]
        self._joint_command.k_qd_p = [k_qd_p]

    def move_gripper(self, goal_joint_angle: float) -> None:
        """Command the gripper to a given joint angle by streaming a command."""
        self._joint_command.position = [goal_joint_angle]
        self._command_pub.publish(self._joint_command)

    def open_and_close(self, duration_sec: float = 1.0, npoints: int = 50) -> None:
        """Open and close the gripper by streaming position commands.

        Args:
            duration_sec (float): Duration of each open and close movement
            npoints (int): number of points to send for each open and close movement.
        """
        dt = duration_sec / npoints
        step_size_rad = (GRIPPER_OPEN_ANGLE - GRIPPER_CLOSE_ANGLE) / npoints
        self._logger.info("Opening...")
        for i in range(npoints):
            self.move_gripper(goal_joint_angle=(GRIPPER_CLOSE_ANGLE + i * step_size_rad))
            time.sleep(dt)
        self._logger.info("Closing...")
        for i in range(npoints):
            self.move_gripper(goal_joint_angle=(GRIPPER_OPEN_ANGLE - i * step_size_rad))
            time.sleep(dt)


@ros_process.main()
def main() -> None:
    """Run the example."""
    example = ExampleGripperStreaming(node=main.node)
    example.open_and_close()
    while True:
        # Get gains to try out from the user.
        k_q_p_str = input(f"Select k_q_p gain (current value: {example.get_k_q_p()}): ")
        k_qd_p_str = input(f"Select k_qd_p gain (current value: {example.get_k_qd_p()}): ")
        print(f"you selected k_q_p={k_q_p_str}, k_qd_p={k_qd_p_str}")
        try:
            k_q_p = float(k_q_p_str)
            k_qd_p = float(k_qd_p_str)
            example.set_gains(k_q_p, k_qd_p)
        except ValueError:
            print("Your inputs could not be converted to floats -- keeping the same gains.")
        # open and close the gripper with the new gains set.
        example.open_and_close()


if __name__ == "__main__":
    main()
