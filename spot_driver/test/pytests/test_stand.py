# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the Stand command.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bosdyn.api.robot_command_pb2 import RobotCommandResponse
from std_srvs.srv import Trigger
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope

from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.usefixtures("spot_node")
def test_stand(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test checks if the "stand" service infrastructure is
    setup correctly.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Send ROS request.
    client = ros.node.create_client(Trigger, "stand")
    future = client.call_async(Trigger.Request())

    # Mock GRPC sever.

    # Serve stand command.
    call = simple_spot.api.RobotCommand.serve(timeout=2.0)
    assert call is not None
    response = RobotCommandResponse()
    response.status = RobotCommandResponse.Status.STATUS_OK
    call.returns(response)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert response.success


@pytest.mark.usefixtures("spot_node")
def test_stand_failed(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    Test what happens when the "stand" commands returns and unkwnown error.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Send ROS request.
    client = ros.node.create_client(Trigger, "stand")
    future = client.call_async(Trigger.Request())

    # Mock GRPC sever.

    # Serve stand command with an unknown error.
    call = simple_spot.api.RobotCommand.serve(timeout=2.0)
    assert call is not None
    response = RobotCommandResponse()
    response.status = RobotCommandResponse.Status.STATUS_UNKNOWN
    call.returns(response)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert not response.success
