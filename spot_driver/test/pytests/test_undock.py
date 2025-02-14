# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the Undock command.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bosdyn.api.docking.docking_pb2 import DockingCommandFeedbackResponse, DockingCommandResponse
from std_srvs.srv import Trigger
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope

from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.usefixtures("spot_node")
def test_undock(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test checks if the "undock" service infrastructure is
    setup correctly.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Send ROS request.
    client = ros.node.create_client(Trigger, "undock")
    future = client.call_async(Trigger.Request())

    # Mock GRPC sever.

    # Serve undock command.
    undock_call = simple_spot.api.DockingCommand.serve(timeout=2.0)
    assert undock_call is not None
    undock_response = DockingCommandResponse()
    undock_response.status = DockingCommandResponse.Status.STATUS_OK
    undock_call.returns(undock_response)

    # Serve undock command feedback.
    undock_feedback_call = simple_spot.api.DockingCommandFeedback.serve(timeout=2.0)
    assert undock_feedback_call is not None
    undock_feedback_response = DockingCommandFeedbackResponse()
    undock_feedback_response.status = DockingCommandFeedbackResponse.STATUS_AT_PREP_POSE
    undock_feedback_call.returns(undock_feedback_response)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert response.success


@pytest.mark.usefixtures("spot_node")
def test_undock_failed(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    Test what happens when the "undock" command returns an unknown error.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Send ROS request.
    client = ros.node.create_client(Trigger, "undock")
    future = client.call_async(Trigger.Request())

    # Mock GRPC sever.

    # Serve undock command.
    undock_call = simple_spot.api.DockingCommand.serve(timeout=2.0)
    assert undock_call is not None
    undock_response = DockingCommandResponse()
    undock_response.status = DockingCommandResponse.Status.STATUS_UNKNOWN
    undock_call.returns(undock_response)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert not response.success
