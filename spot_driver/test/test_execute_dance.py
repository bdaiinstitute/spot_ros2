# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the Execute Dance command.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.scope import ROSAwareScope
from bosdyn.api.spot.choreography_sequence_pb2 import ExecuteChoreographyResponse, UploadChoreographyResponse
from bosdyn.choreography.client.choreography import load_choreography_sequence_from_txt_file

from spot_msgs.srv import ExecuteDance  # type: ignore
from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.usefixtures("spot_node")
def test_execute_dance(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test checks if the "execute_dance" service infrastructure is
    setup correctly.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Send ROS request.
    client = ros.node.create_client(ExecuteDance, "execute_dance")
    request = ExecuteDance.Request()
    choreography_sequence = load_choreography_sequence_from_txt_file("./resources/line_dance.csq")
    request.choreo_sequence_serialized = choreography_sequence.SerializeToString()
    future = client.call_async(request)

    # Mock GRPC sever.

    # Serve upload_choreography command.
    upload_call = simple_spot.api.UploadChoreography.serve(timeout=2.0)
    assert upload_call is not None
    upload_response = UploadChoreographyResponse()
    upload_call.returns(upload_response)

    # Serve execute_dance command.
    execute_call = simple_spot.api.ExecuteChoreography.serve(timeout=2.0)
    assert execute_call is not None
    execute_response = ExecuteChoreographyResponse()
    execute_response.status = ExecuteChoreographyResponse.Status.STATUS_OK
    execute_call.returns(execute_response)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert response.success
