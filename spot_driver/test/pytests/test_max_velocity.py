# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the Max Velocity command.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope

from spot_msgs.srv import SetVelocity  # type: ignore


@pytest.mark.usefixtures("spot_node")
def test_max_velocity(ros: ROSAwareScope) -> None:
    """
    This integration test checks if the "max_velocity" service infrastructure is
    setup correctly.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Send ROS request.
    client = ros.node.create_client(SetVelocity, "max_velocity")
    future = client.call_async(SetVelocity.Request())

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert response.success
