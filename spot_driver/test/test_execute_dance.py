# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the Execute Dance command.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.scope import ROSAwareScope
from bosdyn.api.spot.choreography_sequence_pb2 import (
    ChoreographySequence,
    ExecuteChoreographyResponse,
    UploadChoreographyResponse,
)
from google.protobuf import text_format

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

    # For test purposes, this choreography must be as fast as possible.
    # Internally, the code calculates the time required to execute
    # all moves and sleeps for the corresponding amount of time.
    # If the choreography takes too long, the test times out.
    data = """
        name: "Line Dance"
        slices_per_minute: 520.0
        moves {
            type: "rotate_body"
            requested_slices: 1
            rotate_body_params {
                rotation {
                    roll {
                        value: -0.1
                    }
                }
                return_to_start_pose {
                }
            }
        }
    """

    # Send ROS request.
    client = ros.node.create_client(ExecuteDance, "execute_dance")
    request = ExecuteDance.Request()
    choreography_sequence = ChoreographySequence()
    text_format.Merge(data, choreography_sequence)
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
