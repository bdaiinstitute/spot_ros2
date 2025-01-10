# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the Robot command.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus, StopCommand
from bosdyn.api.full_body_command_pb2 import FullBodyCommand
from bosdyn.api.robot_command_pb2 import RobotCommandFeedback, RobotCommandFeedbackResponse, RobotCommandResponse
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from rclpy.action import ActionClient
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope

from spot_msgs.action import RobotCommand as RobotCommandAction  # type: ignore
from spot_msgs.srv import RobotCommand as RobotCommandService  # type: ignore
from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.usefixtures("spot_node")
def test_robot_command(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test checks if the "robot_command" action infrastructure is
    setup correctly.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Send a ROS goal.

    action_client = ActionClient(ros.node, RobotCommandAction, "robot_command")
    goal = RobotCommandAction.Goal()
    future = action_client.send_goal_async(goal)

    # Mock GRPC sever.

    # Serve robot command.
    command_call = simple_spot.api.RobotCommand.serve(timeout=2.0)
    assert command_call is not None
    command_response = RobotCommandResponse()
    command_response.robot_command_id = 5
    command_response.status = RobotCommandResponse.Status.STATUS_OK
    command_call.returns(command_response)

    # Wait for the goal.
    assert wait_for_future(future, timeout_sec=2.0)
    goal_handle = future.result()

    # Serve command feedback.
    feedback_call = simple_spot.api.RobotCommandFeedback.serve(timeout=2.0)
    assert feedback_call is not None
    feedback_response = RobotCommandFeedbackResponse(
        feedback=RobotCommandFeedback(
            full_body_feedback=FullBodyCommand.Feedback(
                stop_feedback=StopCommand.Feedback(), status=RobotCommandFeedbackStatus.Status.STATUS_PROCESSING
            )
        )
    )
    feedback_call.returns(feedback_response)

    # Query and wait for the action result.
    result_future = goal_handle.get_result_async()
    assert wait_for_future(result_future, timeout_sec=2.0)

    action_result = result_future.result()
    assert action_result.result.success


@pytest.mark.usefixtures("spot_node")
def test_robot_command_failed(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    Test what happens when the "robot command" goal fails.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Send a ROS goal.

    action_client = ActionClient(ros.node, RobotCommandAction, "robot_command")
    goal = RobotCommandAction.Goal()
    future = action_client.send_goal_async(goal)

    # Mock GRPC sever.

    # Serve robot command with an unknown error.
    command_call = simple_spot.api.RobotCommand.serve(timeout=2.0)
    assert command_call is not None
    command_response = RobotCommandResponse()
    command_response.robot_command_id = 5
    command_response.status = RobotCommandResponse.Status.STATUS_UNKNOWN
    command_call.returns(command_response)

    # Wait for the goal.
    assert wait_for_future(future, timeout_sec=2.0)
    goal_handle = future.result()

    # Query and wait for the action result.
    result_future = goal_handle.get_result_async()
    assert wait_for_future(result_future, timeout_sec=2.0)

    action_result = result_future.result()
    assert not action_result.result.success


@pytest.mark.usefixtures("spot_node")
def test_robot_command_starts(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test checks if the "robot_command" service infrastructure is
    setup correctly.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    command = RobotCommandBuilder.synchro_velocity_command(v_x=1.0, v_y=2.0, v_rot=3.0)

    # Make a ROS service request.
    client = ros.node.create_client(RobotCommandService, "robot_command")
    request = RobotCommandService.Request()
    convert(command, request.command)
    request.duration.sec = 1
    future = client.call_async(request)

    # Mock GRPC sever.

    # Serve robot command.
    call = simple_spot.api.RobotCommand.serve(timeout=2.0)
    assert call is not None
    mobility_command = call.request.command.synchronized_command.mobility_command
    assert mobility_command.se2_velocity_request.velocity.linear.x == 1.0
    assert mobility_command.se2_velocity_request.velocity.linear.y == 2.0
    assert mobility_command.se2_velocity_request.velocity.angular == 3.0
    assert mobility_command.se2_velocity_request.end_time.seconds != 0

    command_response = RobotCommandResponse()
    command_response.robot_command_id = 5
    command_response.status = RobotCommandResponse.Status.STATUS_OK
    call.returns(command_response)

    # Wait for the response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()

    assert response.success
    assert response.robot_command_id == 5


@pytest.mark.usefixtures("spot_node")
def test_robot_command_fails_to_start(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    Test what happens when the "robot command" service request fails.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Make a ROS service request.
    client = ros.node.create_client(RobotCommandService, "robot_command")
    request = RobotCommandService.Request()
    convert(RobotCommandBuilder.synchro_stand_command(), request.command)
    future = client.call_async(request)

    # Mock GRPC sever.

    # Serve robot command.
    call = simple_spot.api.RobotCommand.serve(timeout=2.0)
    assert call is not None
    command_response = RobotCommandResponse()
    command_response.status = RobotCommandResponse.Status.STATUS_NOT_POWERED_ON
    call.returns(command_response)

    # Wait for the response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()

    assert not response.success
