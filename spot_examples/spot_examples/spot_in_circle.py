import argparse
import logging
from typing import Optional

import synchros2.process as ros_process
import synchros2.scope as ros_scope
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME, ODOM_FRAME_NAME
from bosdyn.client.math_helpers import Quat, SE2Pose, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from rclpy.node import Node
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import fqn, namespace_with
import math
from bosdyn.client import math_helpers
#from std_srvs.srv import Trigger

from spot_msgs.action import RobotCommand  # type: ignore
from bosdyn.api import geometry_pb2
from .simple_spot_commander import SimpleSpotCommander

# Where we want the robot to walk to relative to itself



class WalkForward:
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        self._logger = logging.getLogger(fqn(self.__class__))
        node = node or ros_scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use synchros2.process.main?)")
        self._robot_name = robot_name

        self._body_frame_name = namespace_with(self._robot_name, BODY_FRAME_NAME)
        self._vision_frame_name = namespace_with(self._robot_name, VISION_FRAME_NAME)
        self._odom_frame_name = namespace_with(self._robot_name, ODOM_FRAME_NAME)
        self._tf_listener = TFListenerWrapper(node)
        self._tf_listener.wait_for_a_tform_b(self._body_frame_name, self._vision_frame_name)
        self._robot = SimpleSpotCommander(self._robot_name, node)
        self._robot_command_client = ActionClientWrapper(
            RobotCommand, namespace_with(self._robot_name, "robot_command"), node
        )
        #self.cli_unstow = self.node.create_client(Trigger, namespace_with(robot_name, "arm_unstow"))

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
        self.use_arm = True

        if self.use_arm:
            print("[INFORMATION] You are using the arm")
            #self.cli_unstow.call_async(Trigger.Request())
            #result = self._robot.command("arm_unstow")
            #if not result.success:
            #    self._logger.error("Robot did not stand message was " + result.message)
            #    return False
            #self._logger.info("Successfully unstowed up.")
        return True

    def get_me_a_circle(self, radius = 1.0, steps = 4):
        dx_all = []
        dy_all =[]
        dyaw_all = []

        angle_subtended = 2 * math.pi / steps #(360 / n)
        chord_length = 2 * radius * math.sin(angle_subtended / 2)

        for i in range(steps):
            dx = chord_length
            dy = 0.0
            #dyaw = -math.degrees(angle_subtended)  # rotate counter-clockwise
            dyaw = -angle_subtended

            dx_all.append(dx)
            dy_all.append(dy)
            dyaw_all.append(dyaw)
        return dx_all, dy_all, dyaw_all
    def gaze_at_center(self, robot_command_client, gaze_target_in_odom):
        """Command the arm to gaze at a fixed point in odom frame (0, -1.2, 0)."""
        from bosdyn.client.robot_command import block_until_arm_arrives

        # x, y, z = radius, 0.0, 0.05
        x, y, z = gaze_target_in_odom.x, gaze_target_in_odom.y, gaze_target_in_odom.z  # Gaze target in odom frame (right side of robot at origin)
        # x = 0.0
        # y = -1.2
        # z = 0.0
        print("x", x)
        print("y", y)
        print("z", z)
        #import pdb; pdb.set_trace()
        gaze_cmd = RobotCommandBuilder.arm_gaze_command(x, y, z, ODOM_FRAME_NAME)
        gripper_cmd = RobotCommandBuilder.claw_gripper_open_command()
        gaze_robot_cmd = RobotCommandBuilder.build_synchro_command(gripper_cmd, gaze_cmd)

        action_goal = RobotCommand.Goal()
        convert(gaze_robot_cmd, action_goal.command)
        self._robot_command_client.send_goal_and_wait("gaze", action_goal)
        #cmd_id = self._robot_command_client.robot_command(gaze_robot_cmd)
        print(f"Gazing at fixed point in odom: ({x:.2f}, {y:.2f}, {z:.2f})")
        #block_until_arm_arrives(robot_command_client, cmd_id, timeout_sec=3.0)

    def base_movement(self, dx, dy, dyaw) -> None:
        self._logger.info("Moving the base")

        body_tform_goal = SE2Pose(x=dx, y=dy, angle=dyaw) #the angle here needs to be in radian
        
        #world_t_robot = self._tf_listener.lookup_a_tform_b(self._vision_frame_name, self._body_frame_name)
        odom_t_robot = self._tf_listener.lookup_a_tform_b(self._odom_frame_name, self._body_frame_name)
        world_t_robot_se2 = SE3Pose(
            odom_t_robot.transform.translation.x,
            odom_t_robot.transform.translation.y,
            odom_t_robot.transform.translation.z,
            Quat(
                odom_t_robot.transform.rotation.w,
                odom_t_robot.transform.rotation.x,
                odom_t_robot.transform.rotation.y,
                odom_t_robot.transform.rotation.z,
            ),
        ).get_closest_se2_transform()
        #ROBOT_T_GOAL = SE2Pose(1.0, 0.0, 0.0)
        odom_t_goal = world_t_robot_se2 * body_tform_goal
        
        proto_goal = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=odom_t_goal.x,
            goal_y=odom_t_goal.y,
            goal_heading=odom_t_goal.angle,
            frame_name=ODOM_FRAME_NAME,  # use Boston Dynamics' frame conventions
        )
        action_goal = RobotCommand.Goal()
        convert(proto_goal, action_goal.command)
        self._robot_command_client.send_goal_and_wait("walk_forward", action_goal)
        self._logger.info("Successfully walked forward")



    def spot_in_a_circle(self) -> None:
        self._logger.info("Moving in a circle")

        radius = 1.2
        steps = 12
        dx_all, dy_all, dyaw_all = self.get_me_a_circle(radius=radius, steps=steps)

        if self.use_arm:
            #x,y,z is where you want to gaze in robot body frame
            x, y, z = 0.0, -radius, 0.1 #these are in body frame, we will convert later
            hand_pos_rt_body = geometry_pb2.Vec3(x=x, y=y, z=z)
            body_Q_hand = geometry_pb2.Quaternion(w=1, x=0, y=0, z=0)
            body_t_hand = geometry_pb2.SE3Pose(position=hand_pos_rt_body, rotation=body_Q_hand)

            hand_pos_rt_odom = self._tf_listener.lookup_a_tform_b(self._odom_frame_name, self._body_frame_name)
            odom_T_body_se3 = math_helpers.SE3Pose(
                hand_pos_rt_odom.transform.translation.x,
                hand_pos_rt_odom.transform.translation.y,
                hand_pos_rt_odom.transform.translation.z,
                math_helpers.Quat(
                    hand_pos_rt_odom.transform.rotation.w,
                    hand_pos_rt_odom.transform.rotation.x,
                    hand_pos_rt_odom.transform.rotation.y,
                    hand_pos_rt_odom.transform.rotation.z,
                ),
            )

            gaze_target_in_odom = odom_T_body_se3 * math_helpers.SE3Pose.from_proto(body_t_hand)
            
            #import pdb; pdb.set_trace()
            
            #self._logger.info("Successfully gazed at center")

        for i in range(len(dx_all)):
            try:
                
                if self.use_arm:
                    self.gaze_at_center(self._robot_command_client, gaze_target_in_odom)
                    #gaze_at_center(robot_command_client, gaze_target_in_odom)
                # return relative_move(options.dx, options.dy, math.radians(options.dyaw), options.frame,
                #                      robot_command_client, robot_state_client, stairs=options.stairs)
                self.base_movement(dx_all[i], dy_all[i], dyaw_all[i])
                                

                
                
            finally:
                # Send a Stop at the end, regardless of what happened.
                result = self._robot.command("stop")
                if not result.success:
                    self._logger.error("Unable to make the robot stop, message was " + result.message)





def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> int:
    goto = WalkForward(args.robot, main.node)
    goto.initialize_robot()
    #goto.walk_forward_with_world_frame_goal()
    goto.spot_in_a_circle()
    return 0


if __name__ == "__main__":
    exit(main())
