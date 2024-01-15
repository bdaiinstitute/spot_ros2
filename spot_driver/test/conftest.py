# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Module containing test fixtures.
Pytest automatically discovers all fixtures defined in the file "conftest.py".
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

# When a test needs a fixture, it must specify the fixture name as a parameter.
# In doings so, Pylint raises an incorrect warning about a name being redefined,
# warning that we want disabled.
# pylint: disable=redefined-outer-name

import typing

import bdai_ros2_wrappers.scope as ros_scope
import domain_coordinator
import grpc
import pytest
import rclpy
from bdai_ros2_wrappers.scope import ROSAwareScope
from bosdyn.api.power_pb2 import PowerCommandRequest, PowerCommandResponse, PowerCommandStatus
from bosdyn.api.robot_command_pb2 import RobotCommandResponse

import spot_wrapper.testing
from spot_driver.spot_ros2 import SpotROS
from spot_wrapper.testing.fixtures import SpotFixture
from spot_wrapper.testing.mocks import MockSpot


# pylint: disable=invalid-name,unused-argument
@spot_wrapper.testing.fixture
class simple_spot(MockSpot):
    """
    This is a factory that returns an instance of the class MockSpot,
    served by a local GRPC server.
    When the class is disposed, the GRPC server is shut down.
    The MockSpot and the GRPC server are used to handle calls to a simulated
    Spot robot.
    """

    def PowerCommand(self, request: PowerCommandRequest, context: grpc.ServicerContext) -> PowerCommandResponse:
        """
        Dummy implementation of the PowerCommand command.
        """
        response = PowerCommandResponse()
        response.status = PowerCommandStatus.STATUS_SUCCESS
        return response


# pylint: enable=invalid-name,unused-argument


@pytest.fixture
def ros() -> typing.Iterator[ROSAwareScope]:
    """
    This method is a generator function that returns a different ROS2 scope
    each time it is invoked, to handle a ROS2 context lifecycle.
    """
    with domain_coordinator.domain_id() as domain_id:  # to ensure node isolation
        with ros_scope.top(global_=True, namespace="fixture", domain_id=domain_id) as top:
            yield top


@pytest.fixture
def spot_node(ros: ROSAwareScope, simple_spot: SpotFixture) -> typing.Iterator[SpotROS]:
    """
    This method is a generator function that returns a different SpotROS
    node each time it is invoked.
    The node is assigned and executed inside the given ROS context, and it is
    destroyed when the context goes out of scope.
    """
    parameter_overrides = [
        rclpy.parameter.Parameter("username", rclpy.Parameter.Type.STRING, "spot"),
        rclpy.parameter.Parameter("password", rclpy.Parameter.Type.STRING, "spot"),
        rclpy.parameter.Parameter("hostname", rclpy.Parameter.Type.STRING, simple_spot.address),
        rclpy.parameter.Parameter("port", rclpy.Parameter.Type.INTEGER, simple_spot.port),
        rclpy.parameter.Parameter("spot_name", rclpy.Parameter.Type.STRING, simple_spot.api.name),
        # disable camera publishing for now
        rclpy.parameter.Parameter("publish_rgb", rclpy.Parameter.Type.BOOL, False),
        rclpy.parameter.Parameter("publish_depth", rclpy.Parameter.Type.BOOL, False),
    ]
    with ros.managed(SpotROS, parameter_overrides=parameter_overrides) as node:
        try:
            yield node
        finally:
            # succeed sit command on destruction
            response = RobotCommandResponse()
            response.status = RobotCommandResponse.Status.STATUS_OK  # pylint: disable=no-member
            simple_spot.api.RobotCommand.future.returns(response)


# @pytest.fixture
# def power_on(ros: ROSAwareScope, simple_spot: SpotFixture, spot_node: SpotROS) -> None:
#     """
#     This fixture claims and powers on the Spot mock robot.
#     The SpotFixture creates a mock of the Spot robot GRPC server, but to
#     ise it for additional testing we still need to claim it and power it on.

#     Args:
#         ros: A ROS2 scope that can be used to create clients.
#         simple_spot: a fake Spot robot running on a local GRPC server.
#         spot_node: the main ROS2 node with all services, subscribers,
#             publishers and actions configured.
#     """

#     # Claim.
#     claim_client = ros.node.create_client(Trigger, "claim")
#     future = claim_client.call_async(Trigger.Request())
#     assert wait_for_future(future, timeout_sec=2.0)
#     response = future.result()
#     assert response is not None
#     assert response.success

#     # Power on.
#     power_on_client = ros.node.create_client(Trigger, "power_on")
#     future = power_on_client.call_async(Trigger.Request())
#     call = simple_spot.api.PowerCommand.serve(timeout=2.0)
#     assert call is not None
#     assert call.request.request == PowerCommandRequest.Request.REQUEST_ON_MOTORS
#     power_state = simple_spot.api.robot_state.power_state
#     power_state.motor_power_state = PowerState.MotorPowerState.MOTOR_POWER_STATE_ON
#     response = PowerCommandResponse()
#     response.status = PowerCommandStatus.STATUS_SUCCESS
#     call.returns(response)
#     assert wait_for_future(future, timeout_sec=2.0)
#     response = future.result()
#     assert response.success
