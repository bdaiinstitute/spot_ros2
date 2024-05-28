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
from bosdyn.api.mission.mission_pb2 import (
    GetInfoResponse,
    GetStateResponse,
    LoadMissionRequest,
    LoadMissionResponse,
    PlayMissionRequest,
    PlayMissionResponse,
    RestartMissionRequest,
    RestartMissionResponse,
)
from std_srvs.srv import Trigger

from spot_msgs.srv import (  # type: ignore
    GetMissionInfo,
    GetMissionState,
    LoadMission,
    PauseMission,
    PlayMission,
    RestartMission,
    StopMission,
)
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

    # Test LoadMission
    client = ros.node.create_client(LoadMission, "load_mission")
    assert client.wait_for_service(timeout_sec=2.0)
    load_req = LoadMission.Request(LoadMissionRequest())
    future = client.call_async(load_req)

    call = simple_spot.api.LoadMission.serve(timeout=2.0)
    assert call is not None

    response = LoadMissionResponse()
    response.status = LoadMissionResponse.Status.STATUS_OK
    call.returns(response)

    assert wait_for_future(future, timeout_sec=2.0)
    result = future.result()
    assert response.success

    # Test PlayMission
    client = ros.node.create_client(PlayMission, "play_mission")
    assert client.wait_for_service(timeout_sec=2.0)
    load_req = PlayMission.Request(PlayMissionRequest())
    future = client.call_async(load_req)

    call = simple_spot.api.PlayMission.serve(timeout=2.0)
    assert call is not None

    response = PlayMissionResponse()
    response.status = PlayMissionResponse.Status.STATUS_OK
    call.returns(response)

    assert wait_for_future(future, timeout_sec=2.0)
    result = future.result()
    assert result.success

    # Test PauseMission
    client = ros.node.create_client(PauseMission, "pause_mission")
    assert client.wait_for_service(timeout_sec=2.0)
    load_req = Trigger.Request()
    future = client.call_async(load_req)

    call = simple_spot.api.PauseMission.serve(timeout=2.0)
    assert call is not None

    response = Trigger.Response()
    response.success = True
    call.returns(response)

    assert wait_for_future(future, timeout_sec=2.0)
    result = future.result()
    assert result.success

    # Test RestartMission
    client = ros.node.create_client(RestartMission, "restart_mission")
    assert client.wait_for_service(timeout_sec=2.0)
    load_req = RestartMission.Request(RestartMissionRequest())
    future = client.call_async(load_req)

    call = simple_spot.api.RestartMission.serve(timeout=2.0)
    assert call is not None

    response = RestartMissionResponse()
    response.status = RestartMissionResponse.Status.STATUS_OK
    call.returns(response)

    assert wait_for_future(future, timeout_sec=2.0)
    result = future.result()
    assert result.success

    # Test StopMission
    client = ros.node.create_client(StopMission, "stop_mission")
    assert client.wait_for_service(timeout_sec=2.0)
    load_req = Trigger.Request()
    future = client.call_async(load_req)

    call = simple_spot.api.RestartMission.serve(timeout=2.0)
    assert call is not None

    response = RestartMission.Response()
    response.success = True
    call.returns(response)

    assert wait_for_future(future, timeout_sec=2.0)
    result = future.result()
    assert result.success

    # Test GetMissionInfo
    client = ros.node.create_client(GetMissionInfo, "get_mission_info")
    assert client.wait_for_service(timeout_sec=2.0)
    load_req = GetMissionInfo.Request()
    future = client.call_async(load_req)

    call = simple_spot.api.GetMissionInfo.serve(timeout=2.0)
    assert call is not None

    response = GetInfoResponse()
    response.status = GetInfoResponse.Status.STATUS_OK
    call.returns(response)

    assert wait_for_future(future, timeout_sec=2.0)
    result = future.result()
    assert result.success

    # Test GetMissionState
    client = ros.node.create_client(GetMissionState, "get_mission_state")
    assert client.wait_for_service(timeout_sec=2.0)
    load_req = GetMissionState.Request()
    future = client.call_async(load_req)

    call = simple_spot.api.GetMissionState.serve(timeout=2.0)
    assert call is not None

    response = GetStateResponse()
    response.status = GetStateResponse.Status.STATUS_OK
    call.returns(response)

    assert wait_for_future(future, timeout_sec=2.0)
    result = future.result()
    assert result.success
