from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from rclpy.action import ActionServer

from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import Twist, Pose


from bosdyn.api import geometry_pb2, trajectory_pb2, robot_command_pb2, world_object_pb2
from bosdyn.api.geometry_pb2 import Quaternion, SE2VelocityLimit
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client import math_helpers
from google.protobuf.timestamp_pb2 import Timestamp
import tf2_ros

import spot_driver.conversions as conv
from spot_driver.single_goal_action_server import SingleGoalActionServer

from bosdyn_msgs.msg import RobotCommandFeedback
from bosdyn_msgs.msg import RobotState
from spot_msgs.msg import Metrics
from spot_msgs.msg import LeaseArray, LeaseResource
from spot_msgs.msg import FootState, FootStateArray
from spot_msgs.msg import EStopState, EStopStateArray
from spot_msgs.msg import WiFiState
from spot_msgs.msg import PowerState
from spot_msgs.msg import BehaviorFault, BehaviorFaultState
from spot_msgs.msg import SystemFault, SystemFaultState
from spot_msgs.msg import BatteryState, BatteryStateArray
from spot_msgs.msg import Feedback
from spot_msgs.msg import MobilityParams
from spot_msgs.action import NavigateTo
from spot_msgs.action import RobotCommand
from spot_msgs.action import Trajectory
from spot_msgs.srv import ListGraph
from spot_msgs.srv import ListWorldObjects
from spot_msgs.srv import SetLocomotion
from spot_msgs.srv import ClearBehaviorFault
from spot_msgs.srv import SetVelocity

#####DEBUG/RELEASE: RELATIVE PATH NOT WORKING IN DEBUG
# Release
from .ros_helpers import *
from spot_wrapper.wrapper import SpotWrapper, CameraSource

### Debug
# from ros_helpers import *

import logging
import threading

import signal
import sys

MAX_DURATION = 1e6
MOCK_HOSTNAME = "Mock_spot"


class WaitForGoal(object):
    def __init__(self, clock, time, callback=None):
        self._at_goal = False
        self._callback = callback
        self._clock = clock
        self._time = rclpy.time.Duration(seconds=time)
        self._thread = threading.Thread(target=self._run)
        self._thread.start()

    @property
    def at_goal(self):
        return self._at_goal

    def _run(self):
        start_time = self._clock.now()
        while self._clock.now() - start_time < self._time:
            time.sleep(0.05)
        self._at_goal = True


class SpotROS:
    """Parent class for using the wrapper.  Defines all callbacks and keeps the wrapper alive"""

    def __init__(self):
        self.spot_wrapper: SpotWrapper = None
        self.node = None
        self._printed_once = False

        self.callbacks = {}
        """Dictionary listing what callback to use for what data task"""
        self.callbacks["robot_state"] = self.RobotStateCB
        self.callbacks["metrics"] = self.MetricsCB
        self.callbacks["lease"] = self.LeaseCB
        self.callbacks["world_objects"] = self.WorldObjectsCB

    def RobotStateCB(self, results):
        """Callback for when the Spot Wrapper gets new robot state data.
        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        state = self.spot_wrapper.robot_state

        if state:
            ## joint states ##
            joint_state = GetJointStatesFromState(state, self.spot_wrapper)
            self.joint_state_pub.publish(joint_state)

            ## TF ##
            tf_msg = GetTFFromState(state, self.spot_wrapper, self.mode_parent_odom_tf)
            if len(tf_msg.transforms) > 0:
                self.dynamic_broadcaster.sendTransform(tf_msg.transforms)

            # Odom Twist #
            twist_odom_msg = GetOdomTwistFromState(state, self.spot_wrapper)
            self.odom_twist_pub.publish(twist_odom_msg)

            # Odom #
            if self.mode_parent_odom_tf == self.spot_wrapper.frame_prefix + 'vision':
                odom_msg = GetOdomFromState(state, self.spot_wrapper, use_vision=True)
            else:
                odom_msg = GetOdomFromState(state, self.spot_wrapper, use_vision=False)
            self.odom_pub.publish(odom_msg)

            # Feet #
            foot_array_msg = GetFeetFromState(state, self.spot_wrapper)
            self.feet_pub.publish(foot_array_msg)

            # EStop #
            estop_array_msg = GetEStopStateFromState(state, self.spot_wrapper)
            self.estop_pub.publish(estop_array_msg)

            # WIFI #
            wifi_msg = GetWifiFromState(state, self.spot_wrapper)
            self.wifi_pub.publish(wifi_msg)

            # Battery States #
            battery_states_array_msg = GetBatteryStatesFromState(state, self.spot_wrapper)
            self.battery_pub.publish(battery_states_array_msg)

            # Power State #
            power_state_msg = GetPowerStatesFromState(state, self.spot_wrapper)
            self.power_pub.publish(power_state_msg)

            # System Faults #
            system_fault_state_msg = GetSystemFaultsFromState(state, self.spot_wrapper)
            self.system_faults_pub.publish(system_fault_state_msg)

            # Behavior Faults #
            behavior_fault_state_msg = getBehaviorFaultsFromState(state, self.spot_wrapper)
            self.behavior_faults_pub.publish(behavior_fault_state_msg)

    def MetricsCB(self, results):
        """Callback for when the Spot Wrapper gets new metrics data.
        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        metrics = self.spot_wrapper.metrics
        if metrics:
            metrics_msg = Metrics()
            local_time = self.spot_wrapper.robotToLocalTime(metrics.timestamp)
            metrics_msg.header.stamp = Time(sec=local_time.seconds, nanosec=local_time.nanos)

            for metric in metrics.metrics:
                if metric.label == "distance":
                    metrics_msg.distance = metric.float_value
                if metric.label == "gait cycles":
                    metrics_msg.gait_cycles = metric.int_value
                if metric.label == "time moving":
                    # metrics_msg.time_moving = Time(metric.duration.seconds, metric.duration.nanos)
                    duration = Duration(sec=metric.duration.seconds, nanosec=metric.duration.nanos)
                    metrics_msg.time_moving = duration
                if metric.label == "electric power":
                    # metrics_msg.electric_power = Time(metric.duration.seconds, metric.duration.nanos)
                    duration = Duration(sec=metric.duration.seconds, nanosec=metric.duration.nanos)
                    metrics_msg.electric_power = duration
            self.metrics_pub.publish(metrics_msg)

    def LeaseCB(self, results):
        """Callback for when the Spot Wrapper gets new lease data.
        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        lease_array_msg = LeaseArray()
        lease_list = self.spot_wrapper.lease
        if lease_list:
            for resource in lease_list:
                new_resource = LeaseResource()
                new_resource.resource = resource.resource
                new_resource.lease.resource = resource.lease.resource
                new_resource.lease.epoch = resource.lease.epoch

                for seq in resource.lease.sequence:
                    new_resource.lease.sequence.append(seq)

                new_resource.lease_owner.client_name = resource.lease_owner.client_name
                new_resource.lease_owner.user_name = resource.lease_owner.user_name

                lease_array_msg.resources.append(new_resource)

            self.lease_pub.publish(lease_array_msg)

    def WorldObjectsCB(self, results):
        world_objects = self.spot_wrapper.world_objects
        if world_objects:
            ## TF ##
            tf_msg = GetTFFromWorldObjects(world_objects.world_objects, self.spot_wrapper,
                                           self.mode_parent_odom_tf.value)
            if len(tf_msg.transforms) > 0:
                self.dynamic_broadcaster.sendTransform(tf_msg.transforms)

    def publish_camera_images_callback(self):
        result = self.spot_wrapper.get_images_by_cameras(
            [CameraSource(camera_name, ['visual']) for camera_name in self.cameras_used.value])
        for image_entry in result:
            image_msg, camera_info = bosdyn_data_to_image_and_camera_info_msgs(
                image_entry.image_response, self.spot_wrapper.robotToLocalTime, self.spot_wrapper.frame_prefix)
            image_pub = getattr(self, f"{image_entry.camera_name}_image_pub")
            image_info_pub = getattr(self, f"{image_entry.camera_name}_image_info_pub")
            image_pub.publish(image_msg)
            image_info_pub.publish(camera_info)
            self.populate_camera_static_transforms(image_entry.image_response)

    def publish_depth_images_callback(self):
        result = self.spot_wrapper.get_images_by_cameras(
            [CameraSource(camera_name, ['depth']) for camera_name in self.cameras_used.value])
        for image_entry in result:
            image_msg, camera_info = bosdyn_data_to_image_and_camera_info_msgs(
                image_entry.image_response, self.spot_wrapper.robotToLocalTime, self.spot_wrapper.frame_prefix)
            depth_pub = getattr(self, f"{image_entry.camera_name}_depth_pub")
            depth_info_pub = getattr(self, f"{image_entry.camera_name}_depth_info_pub")
            depth_pub.publish(image_msg)
            depth_info_pub.publish(camera_info)
            self.populate_camera_static_transforms(image_entry.image_response)

    def publish_depth_registered_images_callback(self):
        result = self.spot_wrapper.get_images_by_cameras(
            [CameraSource(camera_name, ['depth_registered']) for camera_name in self.cameras_used.value])
        for image_entry in result:
            image_msg, camera_info = bosdyn_data_to_image_and_camera_info_msgs(
                image_entry.image_response, self.spot_wrapper.robotToLocalTime, self.spot_wrapper.frame_prefix)
            depth_registered_pub = getattr(self, f"{image_entry.camera_name}_depth_registered_pub")
            depth_registered_info_pub = getattr(self, f"{image_entry.camera_name}_depth_registered_info_pub")
            depth_registered_pub.publish(image_msg)
            depth_registered_info_pub.publish(camera_info)
            self.populate_camera_static_transforms(image_entry.image_response)

    def service_wrapper(self, name, handler, request, response):
        if self.spot_wrapper is None:
            self.node.get_logger().info(
                'Mock mode: service ' + name + ' successfully called with request ' + str(request))
            response.success = True
            return response
        return handler(request, response)

    def handle_claim(self, request, response):
        """ROS service handler for the claim service"""
        response.success, response.message = self.spot_wrapper.claim()
        return response

    def handle_release(self, request, response):
        """ROS service handler for the release service"""
        response.success, response.message = self.spot_wrapper.release()
        return response

    def handle_stop(self, request, response):
        """ROS service handler for the stop service"""
        response.success, response.message = self.spot_wrapper.stop()
        return response

    def handle_self_right(self, request, response):
        """ROS service handler for the self-right service"""
        response.success, response.message = self.spot_wrapper.self_right()
        return response

    def handle_sit(self, request, response):
        """ROS service handler for the sit service"""
        response.success, response.message = self.spot_wrapper.sit()
        return response

    def handle_stand(self, request, response):
        """ROS service handler for the stand service"""
        response.success, response.message = self.spot_wrapper.stand()
        return response

    def handle_rollover(self, request, response):
        """ROS service handler for the rollover service"""
        response.success, response.message = self.spot_wrapper.battery_change_pose()
        return response

    def handle_power_on(self, request, response):
        """ROS service handler for the power-on service"""
        response.success, response.message = self.spot_wrapper.power_on()
        return response

    def handle_safe_power_off(self, request, response):
        """ROS service handler for the safe-power-off service"""
        response.success, response.message = self.spot_wrapper.safe_power_off()
        return response

    def handle_estop_hard(self, request, response):
        """ROS service handler to hard-eStop the robot.  The robot will immediately cut power to the motors"""
        response.success, response.message = self.spot_wrapper.assertEStop(True)
        return response

    def handle_estop_soft(self, request, response):
        """ROS service handler to soft-eStop the robot.  The robot will try to settle on the ground before cutting
        power to the motors """
        response.success, response.message = self.spot_wrapper.assertEStop(False)
        return response

    def handle_estop_disengage(self, request, response):
        """ROS service handler to disengage the eStop on the robot."""
        response.success, response.message = self.spot_wrapper.disengageEStop()
        return response

    def handle_clear_behavior_fault(self, request, response):
        """ROS service handler for clearing behavior faults"""
        response.success, response.message = self.spot_wrapper.clear_behavior_fault(request.id)
        return response

    def handle_stair_mode(self, request, response):
        """ROS service handler to set a stair mode to the robot."""
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.stair_hint = request.data
            self.spot_wrapper.set_mobility_params(mobility_params)
            response.success = True
            response.message = 'Success'
            return response
        except Exception as e:
            response.success = False
            response.message = 'Error:{}'.format(e)
            return response

    def handle_locomotion_mode(self, request, response):
        """ROS service handler to set locomotion mode"""
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.locomotion_hint = request.locomotion_mode
            self.spot_wrapper.set_mobility_params(mobility_params)
            response.success = True
            response.message = 'Success'
            return response
        except Exception as e:
            response.success = False
            response.message = 'Error:{}'.format(e)
            return response

    def handle_max_vel(self, request, response):
        """
        Handle a max_velocity service call. This will modify the mobility params to have a limit on the maximum
        velocity that the robot can move during motion commands. This affects trajectory commands and velocity
        commands
        Args:
            req: SetVelocityRequest containing requested maximum velocity
        Returns: SetVelocityResponse
        """
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.vel_limit.CopyFrom(
                SE2VelocityLimit(max_vel=math_helpers.SE2Velocity(request.velocity_limit.linear.x,
                                                                  request.velocity_limit.linear.y,
                                                                  request.velocity_limit.angular.z).to_proto()))
            self.spot_wrapper.set_mobility_params(mobility_params)
            response.success = True
            response.message = 'Success'
            return response
        except Exception as e:
            response.success = False
            response.message = 'Error:{}'.format(e)
            return response

    # -1 = failed, 0 = in progress, 1 = succeeded
    def _goal_complete(self, feedback):
        FAILED = -1
        IN_PROGRESS = False
        SUCCESS = True

        if not feedback:
            # NOTE: it can take an iteration for the feedback to get set.
            return IN_PROGRESS

        if feedback.command.command_choice == feedback.command.COMMAND_FULL_BODY_FEEDBACK_SET:
            if (feedback.command.full_body_feedback.status.value !=
                    feedback.command.full_body_feedback.status.STATUS_PROCESSING):
                return IN_PROGRESS
            if (feedback.command.full_body_feedback.feedback.feedback_choice ==
                    feedback.command.full_body_feedback.feedback.FEEDBACK_STOP_FEEDBACK_SET):
                return SUCCESS
            elif (feedback.command.full_body_feedback.feedback.feedback_choice ==
                  feedback.command.full_body_feedback.feedback.FEEDBACK_FREEZE_FEEDBACK_SET):
                return SUCCESS
            elif (feedback.command.full_body_feedback.feedback.feedback_choice ==
                  feedback.command.full_body_feedback.feedback.FEEDBACK_SELFRIGHT_FEEDBACK_SET):
                return (feedback.command.full_body_feedback.feedback.selfright_feedback.status.value ==
                        feedback.command.full_body_feedback.feedback.selfright_feedback.status.STATUS_COMPLETED)
            elif (feedback.command.full_body_feedback.feedback.feedback_choice ==
                  feedback.command.full_body_feedback.feedback.FEEDBACK_SAFE_POWER_OFF_FEEDBACK_SET):
                return (feedback.command.full_body_feedback.feedback.safe_power_off_feedback.status.value ==
                        feedback.command.full_body_feedback.feedback.safe_power_off_feedback.status.STATUS_POWERED_OFF)
            elif (feedback.command.full_body_feedback.feedback.feedback_choice ==
                  feedback.command.full_body_feedback.feedback.FEEDBACK_BATTERY_CHANGE_POSE_FEEDBACK_SET):
                if (feedback.command.full_body_feedback.feedback.battery_change_pose_feedback.status.value ==
                        feedback.command.full_body_feedback.feedback.battery_change_pose_feedback.status.STATUS_COMPLETED):
                    return SUCCESS
                if (feedback.command.full_body_feedback.feedback.battery_change_pose_feedback.status.value ==
                        feedback.command.full_body_feedback.feedback.battery_change_pose_feedback.status.STATUS_FAILED):
                    return FAILED
                return IN_PROGRESS
            elif (feedback.command.full_body_feedback.feedback.feedback_choice ==
                  feedback.command.full_body_feedback.feedback.FEEDBACK_PAYLOAD_ESTIMATION_FEEDBACK_SET):
                if (feedback.command.full_body_feedback.feedback.payload_estimation_feedback.status.value ==
                        feedback.command.full_body_feedback.feedback.payload_estimation_feedback.status.STATUS_COMPLETED):
                    return SUCCESS
                if (feedback.command.full_body_feedback.feedback.payload_estimation_feedback.status.value ==
                        feedback.command.full_body_feedback.feedback.payload_estimation_feedback.status.STATUS_SMALL_MASS):
                    return SUCCESS
                if (feedback.command.full_body_feedback.feedback.payload_estimation_feedback.status.value ==
                        feedback.command.full_body_feedback.feedback.payload_estimation_feedback.status.STATUS_ERROR):
                    return FAILED
                return IN_PROGRESS
            elif (feedback.command.full_body_feedback.feedback.feedback_choice ==
                  feedback.command.full_body_feedback.feedback.FEEDBACK_CONSTRAINED_MANIPULATION_FEEDBACK_SET):
                if (feedback.command.full_body_feedback.feedback.constrained_manipulation_feedback.status.value ==
                        feedback.command.full_body_feedback.feedback.constrained_manipulation_feedback.status.
                                STATUS_RUNNING):
                    return IN_PROGRESS
                return FAILED
            else:
                return IN_PROGRESS

        elif feedback.command.command_choice == feedback.command.COMMAND_SYNCHRONIZED_FEEDBACK_SET:
            # The idea here is that a synchronized command can have arm, mobility, and/or gripper
            # sub-commands in it.  The total command isn't done until all sub-commands are satisfied.
            # So if any one of the sub-commands is still in progress, it short-circuits out as
            # IN_PROGRESS.  And if it makes it to the bottom of the function, then all components
            # must be satisfied, and it returns SUCCESS.
            # One corner case to know about is that the commands that don't provide feedback, such
            # as a velocity command will return SUCCESS.  This allows you to use them more effectively
            # with other commands.  For example if you wanted to move the arm with some velocity
            # while the mobility is going to some SE2 trajectory then that will work.

            sync_feedback = feedback.command.synchronized_feedback
            if sync_feedback.arm_command_feedback_is_set == True:
                arm_feedback = sync_feedback.arm_command_feedback
                if (arm_feedback.status.value == arm_feedback.status.STATUS_COMMAND_OVERRIDDEN
                    or arm_feedback.status.value == arm_feedback.status.STATUS_COMMAND_TIMED_OUT
                    or arm_feedback.status.value == arm_feedback.status.STATUS_ROBOT_FROZEN
                    or arm_feedback.status.value == arm_feedback.status.STATUS_INCOMPATIBLE_HARDWARE):
                    return FAILED
                if (arm_feedback.feedback.feedback_choice == arm_feedback.feedback.FEEDBACK_ARM_CARTESIAN_FEEDBACK_SET):
                    if (arm_feedback.feedback.arm_cartesian_feedback.status.value != arm_feedback.feedback.arm_cartesian_feedback.status.STATUS_TRAJECTORY_COMPLETE):
                        return IN_PROGRESS
                elif (arm_feedback.feedback.feedback_choice == arm_feedback.feedback.FEEDBACK_ARM_JOINT_MOVE_FEEDBACK_SET):
                    if (arm_feedback.feedback.arm_joint_move_feedback.status.value != arm_feedback.feedback.arm_joint_move_feedback.status.STATUS_COMPLETE):
                        return IN_PROGRESS
                elif (arm_feedback.feedback.feedback_choice == arm_feedback.feedback.FEEDBACK_NAMED_ARM_POSITION_FEEDBACK_SET):
                    if (arm_feedback.feedback.named_arm_position_feedback.status.value != arm_feedback.feedback.named_arm_position_feedback.status.STATUS_COMPLETE):
                        return IN_PROGRESS
                elif (arm_feedback.feedback.feedback_choice == arm_feedback.feedback.FEEDBACK_ARM_VELOCITY_FEEDBACK_SET):
                    self.node.get_logger().warn('WARNING: ArmVelocityCommand provides no feedback')
                    pass # May return SUCCESS below
                elif (arm_feedback.feedback.feedback_choice == arm_feedback.feedback.FEEDBACK_ARM_GAZE_FEEDBACK_SET):
                    if (arm_feedback.feedback.arm_gaze_feedback.status.value != arm_feedback.feedback.arm_gaze_feedback.status.STATUS_TRAJECTORY_COMPLETE):
                        return IN_PROGRESS
                elif (arm_feedback.feedback.feedback_choice == arm_feedback.feedback.FEEDBACK_ARM_STOP_FEEDBACK_SET):
                    self.node.get_logger().warn('WARNING: Stop command provides no feedback')
                    pass # May return SUCCESS below
                elif (arm_feedback.feedback.feedback_choice == arm_feedback.feedback.FEEDBACK_ARM_DRAG_FEEDBACK_SET):
                    if (arm_feedback.feedback.arm_drag_feedback.status.value != arm_feedback.feedback.arm_drag_feedback.status.STATUS_DRAGGING):
                        return FAILED
                elif (arm_feedback.feedback.feedback_choice == arm_feedback.feedback.FEEDBACK_ARM_IMPEDANCE_FEEDBACK_SET):
                    self.node.get_logger().warn('WARNING: ArmImpedanceCommand provides no feedback')
                    pass # May return SUCCESS below
                else:
                    self.node.get_logger().error('ERROR: unknown arm command type')
                    return IN_PROGRESS

            if sync_feedback.mobility_command_feedback_is_set == True:
                mob_feedback = sync_feedback.mobility_command_feedback
                if (mob_feedback.status.value == mob_feedback.status.STATUS_COMMAND_OVERRIDDEN
                    or mob_feedback.status.value == mob_feedback.status.STATUS_COMMAND_TIMED_OUT
                    or mob_feedback.status.value == mob_feedback.status.STATUS_ROBOT_FROZEN
                    or mob_feedback.status.value == mob_feedback.status.STATUS_INCOMPATIBLE_HARDWARE):
                    return FAILED
                if (mob_feedback.feedback.feedback_choice == mob_feedback.feedback.FEEDBACK_SE2_TRAJECTORY_FEEDBACK_SET):
                    if (mob_feedback.feedback.se2_trajectory_feedback.status.value != mob_feedback.feedback.se2_trajectory_feedback.status.STATUS_AT_GOAL):
                        return IN_PROGRESS
                elif (mob_feedback.feedback.feedback_choice == mob_feedback.feedback.FEEDBACK_SE2_VELOCITY_FEEDBACK_SET):
                    self.node.get_logger().warn('WARNING: Planar velocity commands provide no feedback')
                    pass # May return SUCCESS below
                elif (mob_feedback.feedback.feedback_choice == mob_feedback.feedback.FEEDBACK_SIT_FEEDBACK_SET):
                    if (mob_feedback.feedback.sit_feedback.status.value != mob_feedback.feedback.sit_feedback.status.STATUS_IS_SITTING):
                        return IN_PROGRESS
                elif (mob_feedback.feedback.feedback_choice == mob_feedback.feedback.FEEDBACK_STAND_FEEDBACK_SET):
                    if (mob_feedback.feedback.stand_feedback.status.value != mob_feedback.feedback.stand_feedback.status.STATUS_IS_STANDING):
                        return IN_PROGRESS
                elif (mob_feedback.feedback.feedback_choice == mob_feedback.feedback.FEEDBACK_STANCE_FEEDBACK_SET):
                    if (mob_feedback.feedback.stance_feedback.status.value == mob_feedback.feedback.stance_feedback.status.STATUS_TOO_FAR_AWAY):
                        return FAILED
                    if (mob_feedback.feedback.stance_feedback.status.value != mob_feedback.feedback.stance_feedback.status.STATUS_STANCED):
                        return IN_PROGRESS
                elif (mob_feedback.feedback.feedback_choice == mob_feedback.feedback.FEEDBACK_STOP_FEEDBACK_SET):
                    self.node.get_logger().warn('WARNING: Stop command provides no feedback')
                    pass # May return SUCCESS below
                elif (mob_feedback.feedback.feedback_choice == mob_feedback.feedback.FEEDBACK_FOLLOW_ARM_FEEDBACK_SET):
                    self.node.get_logger().warn('WARNING: FollowArmCommand provides no feedback')
                    pass # May return SUCCESS below
                else:
                    self.node.get_logger().error('ERROR: unknown mobility command type')
                    return IN_PROGRESS

            if sync_feedback.gripper_command_feedback_is_set == True:
                grip_feedback = sync_feedback.gripper_command_feedback
                if (grip_feedback.status.value == grip_feedback.status.STATUS_COMMAND_OVERRIDDEN
                    or grip_feedback.status.value == grip_feedback.status.STATUS_COMMAND_TIMED_OUT
                    or grip_feedback.status.value == grip_feedback.status.STATUS_ROBOT_FROZEN
                    or grip_feedback.status.value == grip_feedback.status.STATUS_INCOMPATIBLE_HARDWARE):
                    return FAILED
                if (grip_feedback.command.command_choice == grip_feedback.command.COMMAND_CLAW_GRIPPER_FEEDBACK_SET):
                    if (grip_feedback.command.claw_gripper_feedback.status.value != grip_feedback.command.claw_gripper_feedback.status.STATUS_AT_GOAL):
                        return IN_PROGRESS
                else:
                    self.node.get_logger().error('ERROR: unknown gripper command type')
                    return IN_PROGRESS

            return SUCCESS

        else:
            self.node.get_logger().error('ERROR: unknown robot command type')
            return IN_PROGRESS

    def _get_robot_command_feedback(self, goal_id):
        feedback = RobotCommandFeedback()
        if goal_id is None:
            feedback.command.command_choice = feedback.command.COMMAND_SYNCHRONIZED_FEEDBACK_SET
            feedback.command.synchronized_feedback.mobility_command_feedback.status.value = \
                feedback.command.synchronized_feedback.mobility_command_feedback.status.STATUS_PROCESSING
            feedback.command.synchronized_feedback.mobility_command_feedback.feedback.feedback_choice = \
                feedback.command.synchronized_feedback.mobility_command_feedback.feedback. \
                    FEEDBACK_SE2_TRAJECTORY_FEEDBACK_SET
            if self._wait_for_goal.at_goal:
                feedback.command.synchronized_feedback.mobility_command_feedback.feedback.se2_trajectory_feedback. \
                    status.value = feedback.command.synchronized_feedback.mobility_command_feedback.feedback. \
                    se2_trajectory_feedback.status.STATUS_AT_GOAL
            else:
                feedback.command.synchronized_feedback.mobility_command_feedback.feedback.se2_trajectory_feedback. \
                    status.value = feedback.command.synchronized_feedback.mobility_command_feedback.feedback. \
                    se2_trajectory_feedback.status.STATUS_GOING_TO_GOAL
        else:
            conv.convert_proto_to_bosdyn_msgs_robot_command_feedback(
                self.spot_wrapper.get_robot_command_feedback(goal_id).feedback, feedback)
        return feedback

    def handle_robot_command(self, goal_handle):
        ros_command = goal_handle.request.command
        proto_command = robot_command_pb2.RobotCommand()
        conv.convert_bosdyn_msgs_robot_command_to_proto(ros_command, proto_command)
        self._wait_for_goal = None
        if not self.spot_wrapper:
            self._wait_for_goal = WaitForGoal(self.node.get_clock(), 2.0)
            goal_id = None
        else:
            success, err_msg, goal_id = self.spot_wrapper.robot_command(proto_command)
            if not success:
                raise Exception(err_msg)

        self.node.get_logger().info('Robot now executing goal ' + str(goal_id))
        # The command is non-blocking but we need to keep this function up in order to interrupt if a
        # preempt is requested and to return success if/when the robot reaches the goal. Also check the is_active to
        # monitor whether the timeout_cb has already aborted the command
        feedback = None
        while (rclpy.ok() and not goal_handle.is_cancel_requested and
               not self._goal_complete(feedback) and goal_handle.is_active):
            feedback = self._get_robot_command_feedback(goal_id)
            feedback_msg = RobotCommand.Feedback(feedback=feedback)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)  # don't use rate here because we're already in a single thread

        # publish a final feedback
        result = RobotCommand.Result()
        if feedback is not None:
            goal_handle.publish_feedback(feedback_msg)
            result.result = feedback
        result.success = self._goal_complete(feedback) == True

        if goal_handle.is_cancel_requested:
            result.success = False
            result.message = "Cancelled"
            goal_handle.abort()
        elif not goal_handle.is_active:
            result.success = False
            result.message = "Cancelled"
            # Don't abort because that's already happened
        elif result.success:
            result.message = "Successfully completed command"
            goal_handle.succeed()
        else:
            result.message = "Failed to complete command"
            goal_handle.abort()
        self._wait_for_goal = False
        if not self.spot_wrapper:
            self.node.get_logger().info("Returning action result " + str(result))
        return result

    def handle_trajectory(self, goal_handle):
        """ROS actionserver execution handler to handle receiving a request to move to a location"""

        if goal_handle.request.target_pose.header.frame_id != 'body':
            goal_handle.abort()
            result = Trajectory.Result()
            result.success = False
            result.message = 'frame_id of target_pose must be \'body\''
            return result

        if goal_handle.request.duration.sec <= 0:
            goal_handle.abort()
            result = Trajectory.Result()
            result.success = False
            result.message = 'duration must be larger than 0'
            return result

        cmd_duration_secs = goal_handle.request.duration.sec*1.0

        resp = self.spot_wrapper.trajectory_cmd(
                        goal_x=goal_handle.request.target_pose.pose.position.x,
                        goal_y=goal_handle.request.target_pose.pose.position.y,
                        goal_heading=math_helpers.Quat(
                            w=goal_handle.request.target_pose.pose.orientation.w,
                            x=goal_handle.request.target_pose.pose.orientation.x,
                            y=goal_handle.request.target_pose.pose.orientation.y,
                            z=goal_handle.request.target_pose.pose.orientation.z
                            ).to_yaw(),
                        cmd_duration=cmd_duration_secs,
                        precise_position=goal_handle.request.precise_positioning,
                        )

        command_start_time = self.node.get_clock().now()

        # Abort the action server if cmd_duration is exceeded - the driver stops but does not provide
        # feedback to indicate this so we monitor it ourselves
        # The trajectory command is non-blocking but we need to keep this function up in order to
        # interrupt if a preempt is requested and to return success if/when the robot reaches the goal.
        # Also check the is_active to
        # monitor whether the timeout has already aborted the command

        #
        # Pre-emp missing in port to ROS2 (ros1: self.trajectory_server.is_preempt_requested())
        #

        ### rate = rclpy.Rate(10)

        try:
            while rclpy.ok() and not self.spot_wrapper.at_goal and goal_handle.is_active:
                feedback = Trajectory.Feedback()
                if self.spot_wrapper.near_goal:
                    if self.spot_wrapper._last_trajectory_command_precise:
                        feedback.feedback = "Near goal, performing final adjustments"
                    else:
                        feedback.feedback = "Near goal"
                else:
                    feedback.feedback = "Moving to goal"

                ###rate.sleep()
                goal_handle.publish_feedback(feedback)
                time.sleep(0.1)

                # check for timeout
                com_dur = self.node.get_clock().now() - command_start_time

                if com_dur.nanoseconds/1e9 > (cmd_duration_secs*10.3):
                    # timeout, quit with failure
                    self.node.get_logger().error("TIMEOUT")
                    feedback = Trajectory.Feedback()
                    feedback.feedback = "Failed to reach goal, timed out"
                    goal_handle.publish_feedback(feedback)
                    goal_handle.abort()

            result = Trajectory.Result()
            result.success = False
            result.message = "timeout"

            # If still active after exiting the loop, the command did not time out
            if goal_handle.is_active:
                #            if self.trajectory_server.is_preempt_requested():
                #                self.trajectory_server.publish_feedback(TrajectoryFeedback("Preempted"))
                #                self.trajectory_server.set_preempted()
                #                self.spot_wrapper.stop()
                #                result.success = False
                #                result.message = 'preempt'

                feedback = Trajectory.Feedback()
                if self.spot_wrapper.at_goal:
                    # self.node.get_logger().error("SUCCESS")
                    feedback.feedback = "Reached goal"
                    goal_handle.publish_feedback(feedback)
                    result.success = True
                    result.message = ''
                    goal_handle.succeed()
                else:
                    # self.node.get_logger().error("FAIL")
                    feedback.feedback = "Failed to reach goal"
                    goal_handle.publish_feedback(feedback)
                    result.success = False
                    result.message = 'not at goal'
                    goal_handle.abort()

        except Exception as e:
            self.node.get_logger().error(f"Exception: {type(e)} - {e}")
            result.success = False
            result.message = f"Exception: {type(e)} - {e}"
        # self.node.get_logger().error(f"RETURN FROM HANDLE: {result}")
        return result

    def cmdVelCallback(self, data):
        """Callback for cmd_vel command"""
        if not self.spot_wrapper:
            self.node.get_logger().info('Mock mode, received command vel ' + str(data))
            return
        self.spot_wrapper.velocity_cmd(data.linear.x, data.linear.y, data.angular.z)

    def bodyPoseCallback(self, data):
        """Callback for cmd_vel command"""
        if not self.spot_wrapper:
            self.node.get_logger().info('Mock mode, received command vel ' + str(data))
            return
        q = Quaternion()
        q.x = data.orientation.x
        q.y = data.orientation.y
        q.z = data.orientation.z
        q.w = data.orientation.w
        position = geometry_pb2.Vec3(z=data.position.z)
        pose = geometry_pb2.SE3Pose(position=position, rotation=q)
        point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
        traj = trajectory_pb2.SE3Trajectory(points=[point])
        body_control = spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)

        mobility_params = self.spot_wrapper.get_mobility_params()
        mobility_params.body_control.CopyFrom(body_control)
        self.spot_wrapper.set_mobility_params(mobility_params)

    def handle_list_graph(self, request, response):
        """ROS service handler for listing graph_nav waypoint_ids"""
        try:
            self.node.get_logger().error(f'handle_list_graph: {request}')
            self.spot_wrapper._clear_graph()
            self.spot_wrapper._upload_graph_and_snapshots(request.upload_filepath)
            response.waypoint_ids = self.spot_wrapper.list_graph(request.upload_filepath)
            self.node.get_logger().error(f'handle_list_graph RESPONSE: {response}')
        except Exception as e:
            self.node.get_logger().error('Exception Error:{}'.format(e))
        return response

    def handle_list_world_objects(self, request, response):
        # For some reason exceptions in service callbacks don't print which makes debugging difficult!
        try:
            return self._handle_list_world_objects(request, response)
        except Exception as e:
            print('In handling list world objects, exception was', traceback.print_exception(e), flush=True)

    def _handle_list_world_objects(self, request, response):
        object_types = [ot.value for ot in request.request.object_type]
        time_start_point = None
        if request.request.timestamp_filter_is_set:
            time_start_point = request.request.timestamp_filter.sec + \
                               float(request.request.timestamp_filter.nanosec) / 1e9
        if not self.spot_wrapper:
            print('Mock return for', object_types, 'after time', time_start_point, flush=True)
            # return a fake list
            proto_response = world_object_pb2.ListWorldObjectResponse()
            world_object = proto_response.world_objects.add()
            world_object.name = 'my_fiducial_3'
            world_object.apriltag_properties.tag_id = 3
            world_object.apriltag_properties.frame_name_fiducial = 'fiducial_3'
            world_object.apriltag_properties.frame_name_fiducial_filtered = 'filtered_fiducial_3'
        else:
            proto_response = self.spot_wrapper.list_world_objects(object_types, time_start_point)
        conv.convert_proto_to_bosdyn_msgs_list_world_object_response(proto_response, response.response)
        return response

    def handle_navigate_to_feedback(self):
        """Thread function to send navigate_to feedback"""
        while rclpy.ok() and self.run_navigate_to:
            localization_state = self.spot_wrapper._graph_nav_client.get_localization_state()
            if localization_state.localization.waypoint_id:
                feedback = NavigateTo.Feedback()
                feedback.waypoint_id = localization_state.localization.waypoint_id
                self.goal_handle.publish_feedback(feedback)
            time.sleep(0.1)
            ## rclpy.Rate(10).sleep()

    def handle_navigate_to(self, goal_handle):
        """ROS service handler to run mission of the robot.  The robot will replay a mission"""
        # create thread to periodically publish feedback
        self.goal_handle = goal_handle
        feedback_thread = threading.Thread(target = self.handle_navigate_to_feedback, args = ())
        self.run_navigate_to = True
        feedback_thread.start()
        # run navigate_to
        resp = self.spot_wrapper.navigate_to(upload_path = goal_handle.request.upload_path,
                                             navigate_to = goal_handle.request.navigate_to,
                                             initial_localization_fiducial = goal_handle.request.initial_localization_fiducial,
                                             initial_localization_waypoint = goal_handle.request.initial_localization_waypoint)
        self.run_navigate_to = False
        feedback_thread.join()

        result = NavigateTo.Result()
        result.success = resp[0]
        result.message = resp[1]
        # check status
        if resp[0]:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def populate_camera_static_transforms(self, image_data):
        """Check data received from one of the image tasks and use the transform snapshot to extract the camera frame
        transforms. This is the transforms from body->frontleft->frontleft_fisheye, for example. These transforms
        never change, but they may be calibrated slightly differently for each robot so we need to generate the
        transforms at runtime.
        Args:
        image_data: Image protobuf data from the wrapper
        """
        # We exclude the odometry frames from static transforms since they are not static. We can ignore the body
        # frame because it is a child of odom or vision depending on the mode_parent_odom_tf, and will be published
        # by the non-static transform publishing that is done by the state callback
        frame_prefix = MOCK_HOSTNAME + '/'
        if self.spot_wrapper is not None:
            frame_prefix = self.spot_wrapper.frame_prefix
        excluded_frames = [self.tf_name_vision_odom.value, self.tf_name_kinematic_odom.value,
                           frame_prefix + "body"]
        excluded_frames = [f[f.rfind('/') + 1:] for f in excluded_frames]
        for frame_name in image_data.shot.transforms_snapshot.child_to_parent_edge_map:
            if frame_name in excluded_frames:
                continue
            parent_frame = image_data.shot.transforms_snapshot.child_to_parent_edge_map.get(
                frame_name).parent_frame_name
            existing_transforms = [(transform.header.frame_id, transform.child_frame_id) for transform in
                                   self.camera_static_transforms]
            if (frame_prefix + parent_frame, frame_prefix + frame_name) in existing_transforms:
                # We already extracted this transform
                continue

            transform = image_data.shot.transforms_snapshot.child_to_parent_edge_map.get(frame_name)
            local_time = self.spot_wrapper.robotToLocalTime(image_data.shot.acquisition_time)
            tf_time = Time(sec=local_time.seconds, nanosec=local_time.nanos)
            static_tf = populateTransformStamped(tf_time, transform.parent_frame_name, frame_name,
                                                 transform.parent_tform_child, frame_prefix)
            self.camera_static_transforms.append(static_tf)
            self.camera_static_transform_broadcaster.sendTransform(self.camera_static_transforms)

    def shutdown(self, sig, frame):
        self.node.get_logger().info("Shutting down ROS driver for Spot")
        self.spot_wrapper.sit()
        self.node_rate.sleep()
        self.spot_wrapper.disconnect()

    def step(self):
        """ Update spot sensors """
        if not self._printed_once:
            self.node.get_logger().info("Driver successfully started!")
            self._printed_once = True
        ### self.node.get_logger().info("Step/Update")
        if rclpy.ok():
            self.spot_wrapper.updateTasks() ############## testing with Robot
            #self.node.get_logger().info("UPDATE TASKS")
            feedback_msg = Feedback()
            if self.spot_wrapper:
                feedback_msg.standing = self.spot_wrapper.is_standing
                feedback_msg.sitting = self.spot_wrapper.is_sitting
                feedback_msg.moving = self.spot_wrapper.is_moving
                id = self.spot_wrapper.id
                try:
                    feedback_msg.serial_number = id.serial_number
                    feedback_msg.species = id.species
                    feedback_msg.version = id.version
                    feedback_msg.nickname = id.nickname
                    feedback_msg.computer_serial_number = id.computer_serial_number
                except:
                    pass
            self.feedback_pub.publish(feedback_msg)
            mobility_params_msg = MobilityParams()
            try:
                mobility_params = self.spot_wrapper.get_mobility_params()
                mobility_params_msg.body_control.position.x = \
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.position.x
                mobility_params_msg.body_control.position.y = \
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.position.y
                mobility_params_msg.body_control.position.z = \
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.position.z
                mobility_params_msg.body_control.orientation.x = \
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.rotation.x
                mobility_params_msg.body_control.orientation.y = \
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.rotation.y
                mobility_params_msg.body_control.orientation.z = \
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.rotation.z
                mobility_params_msg.body_control.orientation.w = \
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.rotation.w
                mobility_params_msg.locomotion_hint = mobility_params.locomotion_hint
                mobility_params_msg.stair_hint = mobility_params.stair_hint
            except Exception as e:
                self.node.get_logger().error('Error:{}'.format(e))
                pass
            self.mobility_params_pub.publish(mobility_params_msg)
            self.node_rate.sleep()


def main(args=None):
    COLOR_END    = '\33[0m'
    COLOR_RED    = '\33[31m'
    COLOR_GREEN  = '\33[32m'
    COLOR_YELLOW = '\33[33m'

    print(COLOR_GREEN + 'Hi from spot_driver.' + COLOR_END)

    spot_ros = SpotROS()
    rclpy.init(args=args)
    """
    Main function for the SpotROS class.  Gets config from ROS and initializes the wrapper.  Holds lease from wrapper
     and updates all async tasks at the ROS rate
    """

    node = rclpy.create_node('spot_ros2')

    spot_ros.node = node
    spot_ros.group = ReentrantCallbackGroup()
    spot_ros.rgb_callback_group = MutuallyExclusiveCallbackGroup()
    spot_ros.depth_callback_group = MutuallyExclusiveCallbackGroup()
    spot_ros.depth_registered_callback_group = MutuallyExclusiveCallbackGroup()
    rate = node.create_rate(100)
    spot_ros.node_rate = rate

    # spot_ros.yaml
    spot_ros.rates = {}
    spot_ros.rates['robot_state'] = 50.0
    spot_ros.rates['metrics'] = 0.04
    spot_ros.rates['lease'] = 1.0
    spot_ros.rates['world_objects'] = 20.0
    spot_ros.rates['front_image'] = 10.0
    spot_ros.rates['side_image'] = 10.0
    spot_ros.rates['rear_image'] = 10.0

    node.declare_parameter('auto_claim', False)
    node.declare_parameter('auto_power_on', False)
    node.declare_parameter('auto_stand', False)

    node.declare_parameter('use_take_lease', True)
    node.declare_parameter('get_lease_on_action', True)
    node.declare_parameter('continually_try_stand', False)

    node.declare_parameter('deadzone', 0.05)
    node.declare_parameter('estop_timeout', 9.0)
    node.declare_parameter('start_estop', False)
    node.declare_parameter('publish_rgb', True)
    node.declare_parameter('publish_depth', True)
    node.declare_parameter('publish_depth_registered', False)
    node.declare_parameter('spot_name', '')

    spot_ros.auto_claim = node.get_parameter('auto_claim')
    spot_ros.auto_power_on = node.get_parameter('auto_power_on')
    spot_ros.auto_stand = node.get_parameter('auto_stand')
    spot_ros.start_estop = node.get_parameter('start_estop')

    spot_ros.use_take_lease = node.get_parameter('use_take_lease')
    spot_ros.get_lease_on_action = node.get_parameter('get_lease_on_action')
    spot_ros.continually_try_stand = node.get_parameter('continually_try_stand')

    spot_ros.publish_rgb = node.get_parameter('publish_rgb')
    spot_ros.publish_depth = node.get_parameter('publish_depth')
    spot_ros.publish_depth_registered = node.get_parameter('publish_depth_registered')

    # This is only done from parameter because it should be passed by the launch file
    spot_ros.name = node.get_parameter('spot_name').value
    if not spot_ros.name:
        spot_ros.name = None

    spot_ros.motion_deadzone = node.get_parameter('deadzone')
    spot_ros.estop_timeout = node.get_parameter('estop_timeout')

    spot_ros.username = get_from_env_and_fall_back_to_param("BOSDYN_CLIENT_USERNAME", node, "username", "user")
    spot_ros.password = get_from_env_and_fall_back_to_param("BOSDYN_CLIENT_PASSWORD", node, "password", "password")
    spot_ros.ip = get_from_env_and_fall_back_to_param("SPOT_IP", node, "hostname", "10.0.0.3")

    spot_ros.camera_static_transform_broadcaster = tf2_ros.StaticTransformBroadcaster(node)
    # Static transform broadcaster is super simple and just a latched publisher. Every time we add a new static
    # transform we must republish all static transforms from this source, otherwise the tree will be incomplete.
    # We keep a list of all the static transforms we already have so they can be republished, and so we can check
    # which ones we already have
    spot_ros.camera_static_transforms = []

    # Spot has 2 types of odometries: 'odom' and 'vision'
    # The former one is kinematic odometry and the second one is a combined odometry of vision and kinematics
    # These params enables to change which odometry frame is a parent of body frame and to change tf names of each odometry frames.
    frame_prefix = ''
    if spot_ros.name is not None:
        frame_prefix = spot_ros.name + '/'
    spot_ros.frame_prefix = frame_prefix
    spot_ros.mode_parent_odom_tf = node.declare_parameter('mode_parent_odom_tf',
                                                          frame_prefix + 'odom') # 'vision' or 'odom'
    spot_ros.tf_name_kinematic_odom = node.declare_parameter('tf_name_kinematic_odom',
                                                             frame_prefix + 'odom')
    spot_ros.tf_name_raw_kinematic = frame_prefix + 'odom'
    spot_ros.tf_name_vision_odom = node.declare_parameter('tf_name_vision_odom', frame_prefix + 'vision')
    spot_ros.tf_name_raw_vision = frame_prefix + 'vision'

    if spot_ros.mode_parent_odom_tf.value != spot_ros.tf_name_raw_kinematic and spot_ros.mode_parent_odom_tf.value != spot_ros.tf_name_raw_vision:
        node.get_logger().error('rosparam "mode_parent_odom_tf" should be "' + frame_prefix + 'odom" or '
                                + frame_prefix + '"vision".')
        return

    # logger
    spot_ros.logger = logging.getLogger('rosout')
    name_str = ''
    if spot_ros.name is not None:
        name_str = ' for ' + spot_ros.name
    node.get_logger().info("Starting ROS driver for Spot" + name_str)
    ############## testing with Robot
    if spot_ros.name == MOCK_HOSTNAME:
        spot_ros.spot_wrapper = None
    else:
        spot_ros.spot_wrapper = SpotWrapper(spot_ros.username, spot_ros.password, spot_ros.ip, spot_ros.name,
                                            spot_ros.logger, spot_ros.start_estop.value, spot_ros.estop_timeout.value,
                                            spot_ros.rates, spot_ros.callbacks, spot_ros.use_take_lease,
                                            spot_ros.get_lease_on_action, spot_ros.continually_try_stand)
        if not spot_ros.spot_wrapper.is_valid:
            return

        all_cameras = ["frontleft", "frontright", "left", "right", "back"]
        has_arm = spot_ros.spot_wrapper.has_arm()
        if has_arm:
            all_cameras.append("hand")
        node.declare_parameter('cameras_used', all_cameras)
        spot_ros.cameras_used = node.get_parameter('cameras_used')

        if spot_ros.publish_rgb.value:
            for camera_name in spot_ros.cameras_used.value:
                setattr(spot_ros, f"{camera_name}_image_pub", node.create_publisher(Image, f"camera/{camera_name}/image", 1))
                setattr(spot_ros, f"{camera_name}_image_info_pub", node.create_publisher(CameraInfo, f"camera/{camera_name}/camera_info", 1))

            node.create_timer(
                1 / spot_ros.rates['front_image'],
                spot_ros.publish_camera_images_callback,
                callback_group=spot_ros.rgb_callback_group,
            )

        if spot_ros.publish_depth.value:
            for camera_name in spot_ros.cameras_used.value:
                setattr(spot_ros, f"{camera_name}_depth_pub", node.create_publisher(Image, f"depth/{camera_name}/image", 1))
                setattr(spot_ros, f"{camera_name}_depth_info_pub", node.create_publisher(CameraInfo, f"depth/{camera_name}/camera_info", 1))

            node.create_timer(
                1 / spot_ros.rates['front_image'],
                spot_ros.publish_depth_images_callback,
                callback_group=spot_ros.depth_callback_group,
            )

        if spot_ros.publish_depth_registered.value:
            for camera_name in spot_ros.cameras_used.value:
                setattr(spot_ros, f"{camera_name}_depth_registered_pub", node.create_publisher(Image, f"depth_registered/{camera_name}/image", 1))
                setattr(spot_ros, f"{camera_name}_depth_registered_pub", node.create_publisher(CameraInfo, f"depth_registered/{camera_name}/camera_info", 1))

            node.create_timer(
                1 / spot_ros.rates['front_image'],
                spot_ros.publish_depth_registered_images_callback,
                callback_group=spot_ros.depth_registered_callback_group,
            )

        node.declare_parameter("has_arm", has_arm)

        # Status Publishers #
        spot_ros.joint_state_pub = node.create_publisher(JointState, 'joint_states', 1)
        spot_ros.dynamic_broadcaster = tf2_ros.TransformBroadcaster(node)
        spot_ros.metrics_pub = node.create_publisher(Metrics, 'status/metrics', 1)
        spot_ros.lease_pub = node.create_publisher(LeaseArray, 'status/leases', 1)
        spot_ros.odom_twist_pub = node.create_publisher(TwistWithCovarianceStamped, 'odometry/twist', 1)
        spot_ros.odom_pub = node.create_publisher(Odometry, 'odometry', 1)
        spot_ros.feet_pub = node.create_publisher(FootStateArray, 'status/feet', 1)
        spot_ros.estop_pub = node.create_publisher(EStopStateArray, 'status/estop', 1)
        spot_ros.wifi_pub = node.create_publisher(WiFiState, 'status/wifi', 1)
        spot_ros.power_pub = node.create_publisher(PowerState, 'status/power_state', 1)
        spot_ros.battery_pub = node.create_publisher(BatteryStateArray, 'status/battery_states', 1)
        spot_ros.behavior_faults_pub = node.create_publisher(BehaviorFaultState, 'status/behavior_faults', 1)
        spot_ros.system_faults_pub = node.create_publisher(SystemFaultState, 'status/system_faults', 1)

        spot_ros.feedback_pub = node.create_publisher(Feedback, 'status/feedback', 1)

        spot_ros.mobility_params_pub = node.create_publisher(MobilityParams, 'status/mobility_params', 1)

        node.create_subscription(Twist, 'cmd_vel', spot_ros.cmdVelCallback, 1, callback_group=spot_ros.group)
        node.create_subscription(Pose, 'body_pose', spot_ros.bodyPoseCallback, 1, callback_group=spot_ros.group)
        node.create_service(
            Trigger, 'claim',
            lambda request, response: spot_ros.service_wrapper('claim', spot_ros.handle_claim, request, response),
            callback_group=spot_ros.group)
        node.create_service(
            Trigger, 'release',
            lambda request, response: spot_ros.service_wrapper('release', spot_ros.handle_release, request, response),
            callback_group=spot_ros.group)
        node.create_service(
            Trigger, "stop",
            lambda request, response: spot_ros.service_wrapper('stop', spot_ros.handle_stop, request, response),
            callback_group=spot_ros.group)
        node.create_service(
            Trigger, "self_right",
            lambda request, response: spot_ros.service_wrapper('self_right', spot_ros.handle_self_right,
                                                               request, response),
            callback_group=spot_ros.group)
        node.create_service(
            Trigger, "sit",
            lambda request, response: spot_ros.service_wrapper('sit', spot_ros.handle_sit, request, response),
            callback_group=spot_ros.group)
        node.create_service(
            Trigger, "stand",
            lambda request, response: spot_ros.service_wrapper('stand', spot_ros.handle_stand, request, response),
            callback_group=spot_ros.group)
        node.create_service(
            Trigger, "rollover",
            lambda request, response: spot_ros.service_wrapper('rollover', spot_ros.handle_rollover, request, response),
            callback_group=spot_ros.group)
        node.create_service(
            Trigger, "power_on",
            lambda request, response: spot_ros.service_wrapper('power_on', spot_ros.handle_power_on, request, response),
            callback_group=spot_ros.group)
        node.create_service(
            Trigger, "power_off",
            lambda request, response: spot_ros.service_wrapper('power_off', spot_ros.handle_safe_power_off,
                                                               request, response),
            callback_group=spot_ros.group)
        node.create_service(
            Trigger, "estop/hard",
            lambda request, response: spot_ros.service_wrapper('estop/hard', spot_ros.handle_estop_hard,
                                                               request, response),
            callback_group=spot_ros.group)
        node.create_service(
            Trigger, "estop/gentle",
            lambda request, response: spot_ros.service_wrapper('estop/gentle', spot_ros.handle_estop_soft,
                                                               request, response),
            callback_group=spot_ros.group)
        node.create_service(
            Trigger, "estop/release",
            lambda request, response: spot_ros.service_wrapper('estop/release', spot_ros.handle_estop_disengage,
                                                               request, response),
            callback_group=spot_ros.group)

        node.create_service(
            SetBool, "stair_mode",
            lambda request, response: spot_ros.service_wrapper('stair_mode', spot_ros.handle_stair_mode,
                                                               request, response),
            callback_group=spot_ros.group)
        node.create_service(
            SetLocomotion, "locomotion_mode",
            lambda request, response: spot_ros.service_wrapper('locomotion_mode', spot_ros.handle_locomotion_mode,
                                                               request, response),
            callback_group=spot_ros.group)
        node.create_service(
            SetVelocity, "max_velocity",
            lambda request, response: spot_ros.service_wrapper('max_velocity', spot_ros.handle_max_vel,
                                                               request, response),
            callback_group=spot_ros.group)
        node.create_service(
            ClearBehaviorFault, "clear_behavior_fault",
            lambda request, response: spot_ros.service_wrapper('clear_behavior_fault',
                                                               spot_ros.handle_clear_behavior_fault,
                                                               request, response),
            callback_group=spot_ros.group)

        node.create_service(
            ListGraph, "list_graph",
            lambda request, response: spot_ros.service_wrapper('list_graph', spot_ros.handle_list_graph,
                                                               request, response),
            callback_group=spot_ros.group)

        # This doesn't use the service wrapper because it's not a trigger and we want different mock responses
        node.create_service(
            ListWorldObjects, "list_world_objects", spot_ros.handle_list_world_objects)

        spot_ros.navigate_as = ActionServer(node, NavigateTo, 'navigate_to', spot_ros.handle_navigate_to,
                                            callback_group=spot_ros.group)
        # spot_ros.navigate_as.start() # As is online

        spot_ros.trajectory_server = ActionServer(node, Trajectory, 'trajectory', spot_ros.handle_trajectory,
                                                  callback_group=spot_ros.group)
        # spot_ros.trajectory_server.start()

        spot_ros.robot_command_server = SingleGoalActionServer(node, RobotCommand, 'robot_command',
                                                               spot_ros.handle_robot_command,
                                                               callback_group=spot_ros.group)

        # Register Shutdown Handle
        # rclpy.on_shutdown(spot_ros.shutdown) ############## Shutdown Handle

        # Wait for an estop to be connected
        if spot_ros.spot_wrapper and not spot_ros.start_estop.value:
            printed = False
            while spot_ros.spot_wrapper.is_estopped():
                if not printed:
                    print(COLOR_YELLOW + 'Waiting for estop to be released.  Make sure you have an active estop.'
                          '  You can acquire an estop on the tablet by choosing "Acquire Cut Motor Power Authority"'
                          ' in the dropdown menu from the power icon.  (This will not power the motors or take the'
                          ' lease.)' + COLOR_END,
                          flush=True)
                    printed = True
                time.sleep(0.5)
            print('Found estop!', flush=True)

        node.create_timer(0.1, spot_ros.step, callback_group=spot_ros.group)

        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(node)

        if spot_ros.spot_wrapper is not None and spot_ros.auto_claim.value:
            spot_ros.spot_wrapper.claim()
            if spot_ros.auto_power_on.value:
                spot_ros.spot_wrapper.power_on()
                if spot_ros.auto_stand.value:
                    spot_ros.spot_wrapper.stand()

        sys.stdout.flush()

        print(f"Spinning ros2_driver")
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass

        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
