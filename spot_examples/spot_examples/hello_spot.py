import argparse
from typing import Optional
import time

import synchros2.process as ros_process
import synchros2.scope as ros_scope


# either 
# from bosdyn.api import geometry_pb2
# from bosdyn.client import math_helpers
# or
# from bosdyn.client.math_helpers import Quat, SE2Pose, SE3Pose

from bosdyn.api import trajectory_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2

from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
import bosdyn.geometry

from bosdyn_msgs.conversions import convert
from rclpy.node import Node
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with
from bosdyn.client import math_helpers
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.util import seconds_to_duration

from spot_msgs.action import RobotCommand  # type: ignore

from .simple_spot_commander import SimpleSpotCommander



def hello_spot(robot_name: Optional[str] = None) -> bool:

    # Set up basic ROS2 utilities for communicating with the driver
    node = ros_scope.node()
    if node is None:
        raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
    logger = node.get_logger()





    """ kinematics setup """

    ## TODO: verify this chunk is correct
    odom_frame_name = namespace_with(robot_name, ODOM_FRAME_NAME)
    grav_aligned_body_frame_name = namespace_with(robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)
    tf_listener = TFListenerWrapper(node)
    tf_listener.wait_for_a_tform_b(odom_frame_name, grav_aligned_body_frame_name)

    robot = SimpleSpotCommander(robot_name, node)
    robot_command_client = ActionClientWrapper(RobotCommand, namespace_with(robot_name, "robot_command"), node)



    """ power on robot """
    
    # Claim robot
    logger.info("Claiming robot")
    result = robot.command("claim")
    if not result.success:
        node.get_logger().error("Unable to claim robot message was " + result.message)
        return False
    logger.info("Claimed robot")

    # Power on robot
    logger.info("Powering on robot")
    result = robot.command("power_on")
    if not result.success:
        logger.error("Unable to power on robot message was " + result.message)
        return False



    """ stand: default """


    logger.info("Standing robot up")
    result = robot.command("stand")
    if not result.success:
        logger.error("Robot did not stand message was " + result.message)
        return False
    logger.info("Successfully stood up.")
    time.sleep(3)






    """ stand: twisted """

    # Tell the robot to stand in a twisted position.
    #
    # The RobotCommandBuilder constructs command messages, which are then
    # issued to the robot using "robot_command" on the command client.
    #
    # In this example, the RobotCommandBuilder generates a stand command
    # message with a non-default rotation in the footprint frame. The footprint
    # frame is a gravity aligned frame with its origin located at the geometric
    # center of the feet. The X axis of the footprint frame points forward along
    # the robot's length, the Z axis points up aligned with gravity, and the Y
    # axis is the cross-product of the two.
    footprint_R_body = bosdyn.geometry.EulerZXY(yaw=0.4, roll=0.0, pitch=0.0)


    ## this
    # cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    # command_client.robot_command(cmd)
    # becomes this
    cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    action_goal = RobotCommand.goal()
    convert(cmd, action_goal.command)
    logger.info("Twisting robot")
    robot_command_client.send_goal_and_wait("twisting_robot", action_goal)


    logger.info('Robot standing twisted.')
    time.sleep(3)



    ## sending commands



    # ## this
    #
    # # Combine the arm and gripper commands into one RobotCommand
    # command = RobotCommandBuilder.build_synchro_command(gripper_command, arm_command)
    #
    # # Send the request
    # cmd_id = command_client.robot_command(command)
    # logger.info('Moving arm to position 1.')
    #
    # # Wait until the arm arrives at the goal.
    # block_until_arm_arrives_with_prints(robot, command_client, cmd_id)
    #
    #
    ## becomes this
    #
    #
    # # Combine the arm and gripper commands into one RobotCommand
    # command = RobotCommandBuilder.build_synchro_command(gripper_command, arm_command)
    #
    # # Convert to a ROS message
    # action_goal = RobotCommand.Goal()
    # convert(command, action_goal.command)
    # # Send the request and wait until the arm arrives at the goal
    # logger.info("Moving arm to position 1.")
    # robot_command_client.send_goal_and_wait("arm_move_one", action_goal)
    #
    #
    #


    """ stand: 3-point trajectory """


    # Now compute an absolute desired position and orientation of the robot body origin.
    # Use the frame helper class to compute the world to gravity aligned body frame transformation.
    # Note, the robot_state used here was cached from before the above yaw stand command,
    # so it contains the nominal stand pose.
    # odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
    #                                  ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)


    ## spot_ros2 replacement
    odom_T_flat_body = tf_listener.lookup_a_tform_b(odom_frame_name, grav_aligned_body_frame_name)

    odom_T_flat_body_se3 = math_helpers.SE3Pose(
        odom_T_flat_body.transform.translation.x,
        odom_T_flat_body.transform.translation.y,
        odom_T_flat_body.transform.translation.z,
        math_helpers.Quat(
            odom_T_flat_body.transform.rotation.w,
            odom_T_flat_body.transform.rotation.x,
            odom_T_flat_body.transform.rotation.y,
            odom_T_flat_body.transform.rotation.z,
        ),
    )

    # Specify a trajectory to shift the body forward followed by looking down, then return to nominal.
    # Define times (in seconds) for each point in the trajectory.
    t1 = 2.5
    t2 = 5.0
    t3 = 7.5

    # Specify the poses as transformations to the cached flat_body pose.
    flat_body_T_pose1 = math_helpers.SE3Pose(x=0.075, y=0, z=0, rot=math_helpers.Quat())
    flat_body_T_pose2 = math_helpers.SE3Pose(
        x=0.0, y=0, z=0, rot=math_helpers.Quat(w=0.9848, x=0, y=0.1736, z=0))
    flat_body_T_pose3 = math_helpers.SE3Pose(x=0.0, y=0, z=0, rot=math_helpers.Quat())

    # Build the points in the trajectory.
    traj_point1 = trajectory_pb2.SE3TrajectoryPoint(
        pose=(odom_T_flat_body_se3 * flat_body_T_pose1).to_proto(),
        time_since_reference=seconds_to_duration(t1))
    traj_point2 = trajectory_pb2.SE3TrajectoryPoint(
        pose=(odom_T_flat_body_se3 * flat_body_T_pose2).to_proto(),
        time_since_reference=seconds_to_duration(t2))
    traj_point3 = trajectory_pb2.SE3TrajectoryPoint(
        pose=(odom_T_flat_body_se3 * flat_body_T_pose3).to_proto(),
        time_since_reference=seconds_to_duration(t3))

    # Build the trajectory proto by combining the points.
    traj = trajectory_pb2.SE3Trajectory(points=[traj_point1, traj_point2, traj_point3])



    ## this
    ## Build a custom mobility params to specify absolute body control
    # body_control = spot_command_pb2.BodyControlParams(
    #     body_pose=spot_command_pb2.BodyControlParams.BodyPose(root_frame_name=ODOM_FRAME_NAME,
    #                                                           base_offset_rt_root=traj))
    

    ## becomes this
    body_control = spot_command_pb2.BodyControlParams(
        body_pose=spot_command_pb2.BodyControlParams.BodyPose(root_frame_name=ODOM_FRAME_NAME,
                                                              base_offset_rt_root=traj))
    
    mobility_params = spot_command_pb2.MobilityParams(body_control=body_control)

    stand_command = RobotCommandBuilder.synchro_stand_command(params=mobility_params)

    stand_command_goal = RobotCommand.Goal()
    convert(stand_command, stand_command_goal.command)
    logger.info('Beginning absolute body control while standing.')
    result = robot_command_client.send_goal_and_wait(
        action_name="hello_spot", goal=stand_command_goal, timeout_sec=10
    )
    logger.info('Finished absolute body control while standing.')




    """ take photo """
    ## TODO: reenable
    # # Capture an image.
    # # Spot has five sensors around the body. Each sensor consists of a stereo pair and a
    # # fisheye camera. The list_image_sources RPC gives a list of image sources which are
    # # available to the API client. Images are captured via calls to the get_image RPC.
    # # Images can be requested from multiple image sources in one call.
    # image_client = robot.ensure_client(ImageClient.default_service_name)
    # sources = image_client.list_image_sources()
    # image_response = image_client.get_image_from_sources(['frontleft_fisheye_image'])
    # _maybe_display_image(image_response[0].shot.image)
    # if config.save or config.save_path is not None:
    #     _maybe_save_image(image_response[0].shot.image, config.save_path)

    ## candidates for spot_ros2 image capturing:

    #
    # def bosdyn_data_to_image_and_camera_info_msgs(
    #     data: image_pb2.ImageResponse, robot_to_local_time: Callable[[Timestamp], Timestamp], frame_prefix: str
    # ) -> Tuple[Union[Image, CompressedImage], CameraInfo]:
    #     """Takes the image and camera data and populates the necessary ROS messages
    #     Args:
    #         data: Image proto
    #         robot_to_local_time: Function to convert the robot time to the local time
    #         frame_prefix: namespace for the published images
    #
    #     Returns:
    #         (tuple):
    #             * Image: message of the image captured
    #             * CameraInfo: message to define the state and config of the camera that took the image
    #     """
    #



    ### Also, there is a topic /<robot_name>/camera/frontleft/image where i can prob take the image from







    #
    #     """ log user comment """
    #
    #     ## TODO: reenable
    #     # # Log a comment.
    #     # # Comments logged via this API are written to the robots test log. This is the best way
    #     # # to mark a log as "interesting". These comments will be available to Boston Dynamics
    #     # # devs when diagnosing customer issues.
    #     # log_comment = 'HelloSpot tutorial user comment.'
    #     # robot.operator_comment(log_comment)
    #     # logger.info('Added comment "%s" to robot log.', log_comment)
    #
    #


    """ should i keep?: power off """

    #
    #
    # # Power the robot off. By specifying "cut_immediately=False", a safe power off command
    # # is issued to the robot. This will attempt to sit the robot before powering off.
    # robot.power_off(cut_immediately=False, timeout_sec=20)
    # assert not robot.is_powered_on(), 'Robot power off failed.'
    # logger.info('Robot safely powered off.')
    #


    return True



def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    hello_spot(args.robot)


if __name__ == "__main__":
    main()
