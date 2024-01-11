# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.scope import ROSAwareScope
from bosdyn.api.robot_command_pb2 import RobotCommandResponse
from std_srvs.srv import Trigger

from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.usefixtures("power_on")
def test_sit(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test checks if the sit service infrastructure is
    setup correctly.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Sit
    sit_client = ros.node.create_client(Trigger, "sit")
    future = sit_client.call_async(Trigger.Request())
    call = simple_spot.api.RobotCommand.serve(timeout=2.0)
    assert call is not None
    response = RobotCommandResponse()
    response.status = RobotCommandResponse.Status.STATUS_OK
    call.returns(response)
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert response.success
