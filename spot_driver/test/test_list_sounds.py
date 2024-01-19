# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the List Sounds command.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.scope import ROSAwareScope
from bosdyn.api.spot_cam.audio_pb2 import ListSoundsResponse

from spot_msgs.srv import ListSounds  # type: ignore
from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.usefixtures("spot_node")
def test_list_sounds(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test checks if the "list_sounds" service infrastructure is
    setup correctly.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Send ROS request.
    client = ros.node.create_client(ListSounds, "list_sounds")
    request = ListSounds.Request()
    future = client.call_async(request)

    # ERROR: spot_cam_wrapper is not set. #########################

    # Mock GRPC sever.

    # Serve "list_sounds" command.
    list_sounds_call = simple_spot.api.ListSounds.serve(timeout=2.0)
    assert list_sounds_call is not None
    list_sounds_response = ListSoundsResponse(sounds=[])
    list_sounds_call.returns(list_sounds_response)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert response.success
