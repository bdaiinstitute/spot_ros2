# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the Dock command.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus, StandCommand
from bosdyn.api.docking.docking_pb2 import DockingCommandFeedbackResponse, DockingCommandResponse
from bosdyn.api.robot_command_pb2 import RobotCommandFeedbackResponse, RobotCommandResponse
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope

from spot_msgs.srv import Dock  # type: ignore
from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.usefixtures("spot_node")
def test_dock(simple_spot: SpotFixture, ros: ROSAwareScope) -> None:
    """
    This integration test checks if the "undock" service infrastructure is
    setup correctly.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # This is just a docking station random id for testing purposes.
    random_dock_id = 500

    # Send ROS request.
    client = ros.node.create_client(Dock, "dock")
    request = Dock.Request()
    request.dock_id = random_dock_id
    future = client.call_async(request)

    # Mock GRPC sever.

    # Serve stand command.
    stand_call = simple_spot.api.RobotCommand.serve(timeout=2.0)
    assert stand_call is not None
    stand_response = RobotCommandResponse()
    stand_response.status = RobotCommandResponse.Status.STATUS_OK
    stand_call.returns(stand_response)

    # Serve stand command feedback.
    stand_feedback_call = simple_spot.api.RobotCommandFeedback.serve(timeout=2.0)
    assert stand_feedback_call is not None
    stand_feedback_response = RobotCommandFeedbackResponse()
    stand_feedback_response.feedback.synchronized_feedback.mobility_command_feedback.stand_feedback.status = (
        StandCommand.Feedback.STATUS_IS_STANDING
    )
    stand_feedback_response.feedback.synchronized_feedback.mobility_command_feedback.status = (
        RobotCommandFeedbackStatus.STATUS_PROCESSING
    )
    stand_feedback_call.returns(stand_feedback_response)

    # Serve dock command.
    dock_call = simple_spot.api.DockingCommand.serve(timeout=2.0)
    assert dock_call is not None
    dock_response = DockingCommandResponse()
    dock_response.status = DockingCommandResponse.Status.STATUS_OK
    dock_call.returns(dock_response)

    # Serve dock command feedback.
    dock_feedback_call = simple_spot.api.DockingCommandFeedback.serve(timeout=2.0)
    assert dock_feedback_call is not None
    dock_feedback_response = DockingCommandFeedbackResponse()
    dock_feedback_response.status = DockingCommandFeedbackResponse.STATUS_DOCKED
    dock_feedback_call.returns(dock_feedback_response)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert response.success


@pytest.mark.usefixtures("spot_node")
def test_dock_with_stand_command_failed(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test what happens when the "stand" command step fails
    as part of the whole "dock" command sequence.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # This is just a docking station random id for testing purposes.
    random_dock_id = 500

    # Send ROS request.
    client = ros.node.create_client(Dock, "dock")
    request = Dock.Request()
    request.dock_id = random_dock_id
    future = client.call_async(request)

    # Mock GRPC sever.

    # Serve stand command and return an unknown error.
    stand_call = simple_spot.api.RobotCommand.serve(timeout=2.0)
    assert stand_call is not None
    stand_response = RobotCommandResponse()
    stand_response.status = RobotCommandResponse.Status.STATUS_UNKNOWN
    stand_call.returns(stand_response)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert not response.success


@pytest.mark.usefixtures("spot_node")
def test_dock_with_dock_command_failed(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test what happens when the "dock" command step fails
    as part of the whole "dock" command sequence.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # This is just a docking station random id for testing purposes.
    random_dock_id = 500

    # Send ROS request.
    client = ros.node.create_client(Dock, "dock")
    request = Dock.Request()
    request.dock_id = random_dock_id
    future = client.call_async(request)

    # Mock GRPC sever.

    # Serve stand command.
    stand_call = simple_spot.api.RobotCommand.serve(timeout=2.0)
    assert stand_call is not None
    stand_response = RobotCommandResponse()
    stand_response.status = RobotCommandResponse.Status.STATUS_OK
    stand_call.returns(stand_response)

    # Serve stand command feedback.
    stand_feedback_call = simple_spot.api.RobotCommandFeedback.serve(timeout=2.0)
    assert stand_feedback_call is not None
    stand_feedback_response = RobotCommandFeedbackResponse()
    stand_feedback_response.feedback.synchronized_feedback.mobility_command_feedback.stand_feedback.status = (
        StandCommand.Feedback.STATUS_IS_STANDING
    )
    stand_feedback_response.feedback.synchronized_feedback.mobility_command_feedback.status = (
        RobotCommandFeedbackStatus.STATUS_PROCESSING
    )
    stand_feedback_call.returns(stand_feedback_response)

    # Serve dock command and return unknown error.
    dock_call = simple_spot.api.DockingCommand.serve(timeout=2.0)
    assert dock_call is not None
    dock_response = DockingCommandResponse()
    dock_response.status = DockingCommandResponse.Status.STATUS_UNKNOWN
    dock_call.returns(dock_response)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert not response.success
