# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the Start Recording State command.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bosdyn.api.spot.choreography_sequence_pb2 import StartRecordingStateResponse
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope

from spot_msgs.srv import ChoreographyStartRecordingState  # type: ignore
from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.usefixtures("spot_node")
def test_start_recording_state(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test checks if the "start_recording_state" service infrastructure is
    setup correctly.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Send ROS request.
    client = ros.node.create_client(ChoreographyStartRecordingState, "start_recording_state")
    request = ChoreographyStartRecordingState.Request()
    future = client.call_async(request)

    # Mock GRPC sever.

    # Serve "start_recording_state" command.
    call = simple_spot.api.StartRecordingState.serve(timeout=2.0)
    assert call is not None
    response = StartRecordingStateResponse()
    response.status = StartRecordingStateResponse.Status.STATUS_OK
    call.returns(response)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert response.success
