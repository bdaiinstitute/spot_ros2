# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the Execute Dance command.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bosdyn.api.spot.choreography_sequence_pb2 import (
    ChoreographySequence,
    ChoreographyStatusResponse,
    ExecuteChoreographyResponse,
    UploadChoreographyResponse,
)
from google.protobuf import text_format
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope

from spot_msgs.action import ExecuteDance  # type: ignore
from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.usefixtures("spot_node")
def test_execute_dance_with_upload(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test checks if the "execute_dance" service infrastructure is
    setup correctly. This test follows the execution path of uploading and
    executing a sequence in a single message.

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

    # Get the lease to interact wtih execute dance
    claim_client = ros.node.create_client(Trigger, "claim")
    resp = claim_client.call(Trigger.Request())
    assert resp.success

    # Send ROS request.
    client = ActionClient(ros.node, ExecuteDance, "execute_dance")
    goal = ExecuteDance.Goal()

    choreography_sequence = ChoreographySequence()
    text_format.Merge(data, choreography_sequence)
    goal.choreo_sequence_serialized = choreography_sequence.SerializeToString()

    future = client.send_goal_async(goal)

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

    # Now send Choreo status so that the feedback knows the dance is done
    call = simple_spot.api.ChoreographyStatus.serve(timeout=2.0)
    assert call is not None
    response = ChoreographyStatusResponse()
    response.status = ChoreographyStatusResponse.Status.STATUS_DANCING
    call.returns(response)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    goal_handle = future.result()
    result_future = goal_handle.get_result_async()
    assert wait_for_future(result_future, timeout_sec=2.0)

    final_result = result_future.result()
    assert final_result.result.success


@pytest.mark.usefixtures("spot_node")
def test_execute_dance_by_name(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test checks if the "execute_dance" service infrastructure is
    setup correctly. This test follows the execution path of first uploading a
    sequence and then executing it by name.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # For test purposes, this choreography must be as fast as possible.
    # Internally, the code calculates the time required to execute
    # all moves and sleeps for the corresponding amount of time.
    # If the choreography takes too long, the test times out.
    dance_name = "Line Dance"

    # Get the lease to interact wtih execute dance
    claim_client = ros.node.create_client(Trigger, "claim")
    resp = claim_client.call(Trigger.Request())
    assert resp.success

    # Send ROS request.
    client = ActionClient(ros.node, ExecuteDance, "execute_dance")
    goal = ExecuteDance.Goal()
    goal.choreo_name = dance_name
    future = client.send_goal_async(goal)

    # Mock GRPC sever.

    # Serve execute_dance command.
    execute_call = simple_spot.api.ExecuteChoreography.serve(timeout=2.0)
    assert execute_call is not None
    execute_response = ExecuteChoreographyResponse()
    execute_response.status = ExecuteChoreographyResponse.Status.STATUS_OK
    execute_call.returns(execute_response)

    # Now send Choreo status so that the feedback knows the dance is done
    call = simple_spot.api.ChoreographyStatus.serve(timeout=2.0)
    assert call is not None
    response = ChoreographyStatusResponse()
    response.status = ChoreographyStatusResponse.Status.STATUS_DANCING
    call.returns(response)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    goal_handle = future.result()
    result_future = goal_handle.get_result_async()
    assert wait_for_future(result_future, timeout_sec=2.0)

    final_result = result_future.result()
    assert final_result.result.success
