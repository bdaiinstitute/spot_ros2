# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the Recorded State to Animation command.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bosdyn.api.data_chunk_pb2 import DataChunk
from bosdyn.api.geometry_pb2 import Quaternion, SE3Pose, Vec3
from bosdyn.api.header_pb2 import CommonError, ResponseHeader
from bosdyn.api.spot.choreography_sequence_pb2 import (
    ArmJointAngles,
    ChoreographyStateLog,
    DownloadRobotStateLogResponse,
    LegJointAngles,
    LoggedFootContacts,
    LoggedJoints,
    LoggedStateKeyFrame,
)
from google.protobuf.timestamp_pb2 import Timestamp
from google.protobuf.wrappers_pb2 import DoubleValue
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope

from spot_msgs.srv import ChoreographyRecordedStateToAnimation  # type: ignore
from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.usefixtures("spot_node")
def test_recorded_state_to_animation(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test checks if the "recorded_state_to_animation" service infrastructure is
    setup correctly.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local
            GRPC server.
    """

    # Send ROS request.
    client = ros.node.create_client(ChoreographyRecordedStateToAnimation, "recorded_state_to_animation")
    request = ChoreographyRecordedStateToAnimation.Request()
    future = client.call_async(request)

    # Mock GRPC sever.

    # We create a ChoreographyStateLog, which represents a sequence of
    # positions (frames) assumed by the robot over time.
    # We serialize it and return it in the response.
    # The response is a stream, and we could chunk the serialized log
    # into multiple payloads. For simplicity, we return a stream with
    # one single payload.

    sequence = ChoreographyStateLog(
        key_frames=[
            LoggedStateKeyFrame(
                joint_angles=LoggedJoints(
                    fl=LegJointAngles(hip_x=0.0, hip_y=0.0, knee=0.0),
                    fr=LegJointAngles(hip_x=0.0, hip_y=0.0, knee=0.0),
                    hl=LegJointAngles(hip_x=0.0, hip_y=0.0, knee=0.0),
                    hr=LegJointAngles(hip_x=0.0, hip_y=0.0, knee=0.0),
                    arm=ArmJointAngles(
                        shoulder_0=DoubleValue(value=0.0),
                        shoulder_1=DoubleValue(value=0.0),
                        elbow_0=DoubleValue(value=0.0),
                        elbow_1=DoubleValue(value=0.0),
                        wrist_0=DoubleValue(value=0.0),
                        wrist_1=DoubleValue(value=0.0),
                    ),
                    gripper_angle=DoubleValue(value=0.0),
                ),
                foot_contact_state=LoggedFootContacts(
                    fr_contact=True, fl_contact=True, hr_contact=True, hl_contact=True
                ),
                animation_tform_body=SE3Pose(
                    position=Vec3(x=0.0, y=0.0, z=0.0), rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                ),
                timestamp=Timestamp(),
            )
        ]
    )
    message = sequence.SerializeToString()
    responses = [
        DownloadRobotStateLogResponse(
            header=ResponseHeader(error=CommonError(code=CommonError.Code.CODE_OK, message="Code OK")),
            status=DownloadRobotStateLogResponse.Status.STATUS_OK,
            chunk=DataChunk(total_size=len(message), data=message),
        )
    ]

    # Serve "download_state_log" command.
    call = simple_spot.api.DownloadRobotStateLog.serve(timeout=2.0)
    assert call is not None
    call.returns(responses)

    # Wait for ROS response.
    assert wait_for_future(future, timeout_sec=2.0)
    response = future.result()
    assert response.success
