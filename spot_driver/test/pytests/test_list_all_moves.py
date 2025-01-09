# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the List All Moves command.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bosdyn.api.spot.choreography_sequence_pb2 import ListAllMovesResponse
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope

from spot_msgs.srv import ListAllMoves  # type: ignore
from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.usefixtures("spot_node")
def test_list_all_moves(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test checks if the "list_all_moves" service infrastructure is
    setup correctly.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Send ROS request.
    client = ros.node.create_client(ListAllMoves, "list_all_moves")
    request = ListAllMoves.Request()
    future = client.call_async(request)

    # Mock GRPC sever.

    # Serve "list_all_moves" command.
    list_moves_call = simple_spot.api.ListAllMoves.serve(timeout=2.0)
    assert list_moves_call is not None
    list_moves_response = ListAllMovesResponse()
    list_moves_call.returns(list_moves_response)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert response.success
