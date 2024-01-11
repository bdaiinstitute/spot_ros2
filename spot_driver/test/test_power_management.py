# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import typing

import bdai_ros2_wrappers.scope as ros_scope
import domain_coordinator
import pytest
import rclpy
from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.scope import ROSAwareScope
from bosdyn.api.power_pb2 import (
    PowerCommandRequest,
    PowerCommandResponse,
    PowerCommandStatus,
)
from bosdyn.api.robot_command_pb2 import RobotCommandResponse
from bosdyn.api.robot_state_pb2 import PowerState
from std_srvs.srv import Trigger

import spot_wrapper.testing
from spot_driver.spot_ros2 import SpotROS
from spot_wrapper.testing.fixtures import SpotFixture
from spot_wrapper.testing.mocks import MockSpot


@spot_wrapper.testing.fixture
class simple_spot(MockSpot):
    pass


@pytest.fixture
def ros() -> typing.Iterator[ROSAwareScope]:
    with domain_coordinator.domain_id() as domain_id:  # to ensure node isolation
        with ros_scope.top(global_=True, namespace="fixture", domain_id=domain_id) as top:
            yield top


@pytest.fixture
def spot_node(ros: ROSAwareScope, simple_spot: SpotFixture) -> typing.Iterator[SpotROS]:
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
            response.status = RobotCommandResponse.Status.STATUS_OK
            simple_spot.api.RobotCommand.future.returns(response)


def test_spot_power_on(ros: ROSAwareScope, simple_spot: SpotFixture, spot_node: SpotROS) -> None:
    claim_client = ros.node.create_client(Trigger, "claim")
    future = claim_client.call_async(Trigger.Request())
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert response is not None
    assert response.success

    power_on_client = ros.node.create_client(Trigger, "power_on")
    future = power_on_client.call_async(Trigger.Request())
    call = simple_spot.api.PowerCommand.serve(timeout=2.0)
    assert call is not None
    assert call.request.request == PowerCommandRequest.Request.REQUEST_ON_MOTORS
    power_state = simple_spot.api.robot_state.power_state
    power_state.motor_power_state = PowerState.MotorPowerState.MOTOR_POWER_STATE_ON
    response = PowerCommandResponse()
    response.status = PowerCommandStatus.STATUS_SUCCESS
    call.returns(response)
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert response.success

    power_off_client = ros.node.create_client(Trigger, "power_off")
    future = power_off_client.call_async(Trigger.Request())
    call = simple_spot.api.RobotCommand.serve(timeout=2.0)
    assert call is not None
    assert call.request.command.HasField("full_body_command")
    assert call.request.command.full_body_command.HasField("safe_power_off_request")
    response = RobotCommandResponse()
    response.status = RobotCommandResponse.Status.STATUS_EXPIRED
    response.message = "Power off request too late"
    call.returns(response)
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert not response.success
    assert "ExpiredError" in response.message
