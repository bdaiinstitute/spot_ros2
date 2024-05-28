# Copyright (c) 2024 The AI Institute LLC. See LICENSE file for more info.

"""
Test for the Mission commands.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.scope import ROSAwareScope
from bosdyn.api.mission.mission_service_pb2_grpc import LoadMissionRequest, RestartMissionRequest
from std_srvs.srv import Trigger

from spot_msgs.srv import (
    GetMissionInfo,
    GetMissionState,
    LoadMission,
    PauseMission,
    PlayMission,
    RestartMission,
    StopMission,
)

# type: ignore
from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.usefixtures("spot_node")
def test_mission_services(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
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
    client = ros.node.create_client(LoadMission, "load_mission")
    assert client.wait_for_service(timeout_sec=2.0)
    load_req = LoadMission.Request(LoadMissionRequest())
    future = client.call_async(load_req)
    assert wait_for_future(future, timeout_sec=2.0)
    result = future.result()
    assert not result.success, result.message

    client = ros.node.create_client(PlayMission, "play_mission")
    assert client.wait_for_service(timeout_sec=2.0)
    load_req = PlayMission.Request()
    future = client.call_async(load_req)
    assert wait_for_future(future, timeout_sec=2.0)
    result = future.result()
    assert not result.success, result.message

    client = ros.node.create_client(PauseMission, "pause_mission")
    assert client.wait_for_service(timeout_sec=2.0)
    load_req = PauseMission.Request()
    future = client.call_async(load_req)
    assert wait_for_future(future, timeout_sec=2.0)
    result = future.result()
    assert not result.success, result.message

    client = ros.node.create_client(RestartMission, "restart_mission")
    assert client.wait_for_service(timeout_sec=2.0)
    load_req = RestartMission.Request(RestartMissionRequest())
    future = client.call_async(load_req)
    assert wait_for_future(future, timeout_sec=2.0)
    result = future.result()
    assert not result.success, result.message

    client = ros.node.create_client(StopMission, "stop_mission")
    assert client.wait_for_service(timeout_sec=2.0)
    load_req = StopMission.Request()
    future = client.call_async(load_req)
    assert wait_for_future(future, timeout_sec=2.0)
    result = future.result()
    assert not result.success, result.message

    client = ros.node.create_client(GetMissionInfo, "get_mission_info")
    assert client.wait_for_service(timeout_sec=2.0)
    load_req = GetMissionInfo.Request()
    future = client.call_async(load_req)
    assert wait_for_future(future, timeout_sec=2.0)
    result = future.result()
    assert not result.success, result.message

    client = ros.node.create_client(GetMissionState, "get_mission_state")
    assert client.wait_for_service(timeout_sec=2.0)
    load_req = GetMissionState.Request()
    future = client.call_async(load_req)
    assert wait_for_future(future, timeout_sec=2.0)
    result = future.result()
    assert not result.success, result.message

    # response = RobotCommandResponse()
    # response.status = RobotCommandResponse.Status.STATUS_OK
    # simple_spot.api.RobotCommand.future.returns(response).repeatedly(2)
    # response = RobotCommandResponse()
    # response.status = RobotCommandResponse.Status.STATUS_BEHAVIOR_FAULT
    # simple_spot.api.RobotCommand.future.returns(response).forever()
