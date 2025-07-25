# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the Stair Mode command.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from std_srvs.srv import SetBool
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope


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
    client = ros.node.create_client(SetBool, "stair_mode")
    future = client.call_async(SetBool.Request())

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert response.success
