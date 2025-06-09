# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the Self Right command.
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
def test_self_right(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test checks if the "sit" service infrastructure is
    setup correctly.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Send ROS request.
    client = ros.node.create_client(Trigger, "self_right")
    future = client.call_async(Trigger.Request())

    # Mock GRPC sever.

    # Serve self_right command.
    call = simple_spot.api.RobotCommand.serve(timeout=2.0)
    assert call is not None
    response = RobotCommandResponse()
    response.status = RobotCommandResponse.Status.STATUS_OK
    call.returns(response)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert response.success
