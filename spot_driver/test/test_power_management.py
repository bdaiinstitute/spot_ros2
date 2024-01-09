# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.


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

from spot_driver.spot_ros2 import SpotROS
from spot_wrapper.testing.fixtures import SpotFixture


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
