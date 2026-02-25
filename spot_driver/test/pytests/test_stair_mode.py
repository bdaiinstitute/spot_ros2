# Copyright (c) 2023-2025 Robotics and AI Institute LLC dba RAI Institute. See LICENSE file for more info.

"""
Test for the Stair Mode command.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bosdyn_spot_api_msgs.msg import MobilityParamsStairsMode
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope

from spot_msgs.srv import SetStairsMode  # type: ignore


@pytest.mark.parametrize("simple_spot", [False], indirect=True)
@pytest.mark.usefixtures("spot_node")
def test_stair_mode(ros: ROSAwareScope) -> None:
    """
    This integration test checks if the "stair_mode" service infrastructure is
    setup correctly.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Send ROS request.
    client = ros.node.create_client(SetStairsMode, "stairs_mode")
    request = SetStairsMode.Request()
    request.stairs_mode.value = MobilityParamsStairsMode.STAIRS_MODE_ON
    future = client.call_async(request)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=pytest.DEFAULT_TIMEOUT)
    response = future.result()
    assert response.success
