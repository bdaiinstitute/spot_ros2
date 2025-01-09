# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the Upload Animation command.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bosdyn.api.spot.choreography_sequence_pb2 import Animation, UploadAnimatedMoveResponse
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope

from spot_msgs.srv import UploadAnimation  # type: ignore
from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.usefixtures("spot_node")
def test_upload_animation(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test checks if the "upload_animation" service infrastructure is
    setup correctly.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Send ROS request.
    client = ros.node.create_client(UploadAnimation, "upload_animation")

    animation = Animation(name="my_animation", animation_keyframes=[], controls_legs=True)
    message = animation.SerializeToString()

    request = UploadAnimation.Request()
    request.animation_proto_serialized = message
    future = client.call_async(request)

    # Mock GRPC sever.

    # Serve upload_animated_move command.
    call = simple_spot.api.UploadAnimatedMove.serve(timeout=2.0)
    assert call is not None
    response = UploadAnimatedMoveResponse()
    response.status = UploadAnimatedMoveResponse.Status.STATUS_OK
    call.returns(response)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert response.success


@pytest.mark.usefixtures("spot_node")
def test_upload_animation_failed(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    Test what happens when the "upload animation" command fails.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Send ROS request.
    client = ros.node.create_client(UploadAnimation, "upload_animation")

    animation = Animation(name="my_animation", animation_keyframes=[], controls_legs=True)
    message = animation.SerializeToString()

    request = UploadAnimation.Request()
    request.animation_proto_serialized = message
    future = client.call_async(request)

    # Mock GRPC sever.

    # Serve upload_animated_move command with an unknonw error.
    call = simple_spot.api.UploadAnimatedMove.serve(timeout=2.0)
    assert call is not None
    response = UploadAnimatedMoveResponse()
    response.status = UploadAnimatedMoveResponse.Status.STATUS_UNKNOWN
    call.returns(response)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert not response.success
