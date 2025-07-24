# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the arm surface contact.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bosdyn.api.arm_surface_contact_service_pb2 import ArmSurfaceContactResponse
from bosdyn.api.header_pb2 import CommonError
from bosdyn_msgs.msg import SE3TrajectoryPoint
from rclpy.action import ActionClient
from rclpy.duration import Duration
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope

from spot_msgs.action import ArmSurfaceContact as ArmSurfaceContactAction  # type: ignore
from spot_wrapper.testing.fixtures import SpotFixture


def _send_goal_and_get_result(
    goal: ArmSurfaceContactAction.Goal, action_client: ActionClient
) -> ArmSurfaceContactAction.Result:
    future = action_client.send_goal_async(goal)
    assert wait_for_future(future, timeout_sec=2.0)
    goal_handle = future.result()

    # Query and wait for the action result.
    result_future = goal_handle.get_result_async()
    assert wait_for_future(result_future, timeout_sec=2.0)
    return result_future.result()


@pytest.mark.parametrize("simple_spot", [True], indirect=True)  # This can only be run when spot has an arm
@pytest.mark.usefixtures("spot_node")
def test_arm_surface_contact_command(ros: ROSAwareScope, simple_spot: SpotFixture) -> None:
    """
    This integration test checks if the "arm_surface_contact_command" action infrastructure is
    setup correctly.

    Args:
        ros: A ROS2 scope that can be used to create clients.
        simple_spot: a programmable fake Spot robot running on a local GRPC server.
    """

    # Send a ROS goal.

    action_client = ActionClient(ros.node, ArmSurfaceContactAction, "arm_surface_contact")
    goal = ArmSurfaceContactAction.Goal()
    goal.command.pose_trajectory_in_task.points.append(
        SE3TrajectoryPoint(time_since_reference=Duration(seconds=0.1).to_msg())
    )
    goal.command.pose_trajectory_in_task.points.append(
        SE3TrajectoryPoint(time_since_reference=Duration(seconds=0.2).to_msg())
    )

    # Serve the response
    command_response = ArmSurfaceContactResponse()
    simple_spot.api.ArmSurfaceContact.future.returns(command_response)

    # Get the goal back
    action_result = _send_goal_and_get_result(goal, action_client)
    assert action_result.result.success

    # Test a failure
    command_response.header.error.code = CommonError.CODE_INVALID_REQUEST
    simple_spot.api.ArmSurfaceContact.future.returns(command_response)

    action_result = _send_goal_and_get_result(goal, action_client)
    assert not action_result.result.success
