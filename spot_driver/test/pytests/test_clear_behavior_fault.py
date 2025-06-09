# Copyright (c) 2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the Clear Behavior Fault command.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bosdyn.api.robot_command_pb2 import ClearBehaviorFaultResponse
from std_srvs.srv import Trigger
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope

from spot_msgs.srv import ClearBehaviorFault  # type: ignore
from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.usefixtures("spot_node")
def test_clear_behavior_fault(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test checks if the "clear_behavior_fault" service infrastructure is
    setup correctly.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """
    # Satisfy driver prerequisites.
    client = ros.node.create_client(Trigger, "claim")
    assert client.wait_for_service(timeout_sec=2.0)
    future = client.call_async(Trigger.Request())
    assert wait_for_future(future, timeout_sec=2.0)
    result = future.result()
    assert result.success, result.message

    # Send ROS request.
    client = ros.node.create_client(ClearBehaviorFault, "clear_behavior_fault")
    assert client.wait_for_service(timeout_sec=2.0)
    future = client.call_async(ClearBehaviorFault.Request(id=127))

    # Serve fault clear service.
    call = simple_spot.api.ClearBehaviorFault.serve(timeout=5.0)
    assert call is not None
    assert call.request.behavior_fault_id == 127
    response = ClearBehaviorFaultResponse()
    response.status = ClearBehaviorFaultResponse.Status.STATUS_CLEARED
    call.returns(response)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    result = future.result()
    assert result.success, result.message
