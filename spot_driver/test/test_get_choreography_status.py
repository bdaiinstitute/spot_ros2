# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the Get Choreography Status command.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.scope import ROSAwareScope
from bosdyn.api.spot.choreography_sequence_pb2 import ChoreographyStatusResponse

from spot_msgs.srv import GetChoreographyStatus  # type: ignore
from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.usefixtures("spot_node")
def test_get_choreography_status(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test checks if the "get_choreography_status" service infrastructure is
    setup correctly.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Send ROS request.
    client = ros.node.create_client(GetChoreographyStatus, "get_choreography_status")
    request = GetChoreographyStatus.Request()
    future = client.call_async(request)

    # Mock GRPC sever.

    # Serve "get_choreography_status" command.
    call = simple_spot.api.ChoreographyStatus.serve(timeout=2.0)
    assert call is not None
    response = ChoreographyStatusResponse()
    call.returns(response)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert response.success
