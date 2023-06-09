import sys

from rclpy.node import Node

from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME
from bosdyn.client.math_helpers import SE2Pose
from bosdyn.client.robot_command import RobotCommandBuilder

from spot_msgs.action import RobotCommand
import spot_driver.conversions as conv

import rclpy
from bdai_ros2_wrappers.action_client_wrapper import ActionClientWrapper
from utilities.simple_spot_commander import SimpleSpotCommander
from utilities.tf_listener_wrapper import TFListenerWrapper

# Where we want the robot to walk to relative to itself
ROBOT_T_GOAL = SE2Pose(1.0, 0.0, 0.0)

class WalkForward(Node):
    def __init__(self):
        super().__init__('walk_forward')

        self._tf_listener = TFListenerWrapper('walk_forward_tf', wait_for_transform = [BODY_FRAME_NAME,
                                                                                       VISION_FRAME_NAME])
        self._robot = SimpleSpotCommander()
        self._robot_command_client = ActionClientWrapper(RobotCommand, 'robot_command')


    def initialize_robot(self):
        self.get_logger().info('Claiming robot')
        result = self._robot.command('claim')
        if not result.success:
            self.get_logger().error('Unable to claim robot message was ' + result.message)
            return False
        self.get_logger().info('Claimed robot')

        # Stand the robot up.
        self.get_logger().info('Powering robot on')
        result = self._robot.command('power_on')
        if not result.success:
            self.get_logger().error('Unable to power on robot message was ' + result.message)
            return False
        self.get_logger().info('Standing robot up')
        result = self._robot.command('stand')
        if not result.success:
            self.get_logger().error('Robot did not stand message was ' + result.message)
            return False
        self.get_logger().info('Successfully stood up.')
        return True

    def walk_forward_with_world_frame_goal(self):
        self.get_logger().info('Walking forward')
        world_t_robot = self._tf_listener.lookup_a_tform_b(VISION_FRAME_NAME,
                                                           BODY_FRAME_NAME).get_closest_se2_transform()
        world_t_goal = world_t_robot * ROBOT_T_GOAL
        proto_goal = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=world_t_goal.x, goal_y=world_t_goal.y, goal_heading=world_t_goal.angle,
            frame_name=VISION_FRAME_NAME)
        action_goal = RobotCommand.Goal()
        conv.convert_proto_to_bosdyn_msgs_robot_command(proto_goal, action_goal.command)
        self._robot_command_client.send_goal_and_wait(action_goal)
        self.get_logger().info('Successfully walked forward')

    def shutdown(self):
        self._tf_listener.shutdown()

def main():
    rclpy.init()
    goto = WalkForward()
    goto.initialize_robot()
    goto.walk_forward_with_world_frame_goal()
    goto.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main)
