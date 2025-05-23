# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Test for the Robot command.
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

import pytest
from bosdyn.api.arm_surface_contact_service_pb2 import ArmSurfaceContactResponse
from bosdyn_msgs.msg import SE3TrajectoryPoint
from rclpy.action import ActionClient
from rclpy.duration import Duration
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope

from spot_msgs.action import ArmSurfaceContact as ArmSurfaceContactAction  # type: ignore
from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.parametrize("simple_spot", [True], indirect=True)
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
        SE3TrajectoryPoint(time_since_reference=Duration(seconds=0.5).to_msg())
    )
    goal.command.pose_trajectory_in_task.points.append(
        SE3TrajectoryPoint(time_since_reference=Duration(seconds=1.0).to_msg())
    )
    print(goal)
    future = action_client.send_goal_async(goal)

    # Mock GRPC sever.

    # Serve the response
    print(simple_spot.api)
    command_response = ArmSurfaceContactResponse()
    simple_spot.api.ArmSurfaceContact.future.returns(command_response)

    # Wait for the goal.
    assert wait_for_future(future, timeout_sec=2.0)
    goal_handle = future.result()

    # Query and wait for the action result.
    result_future = goal_handle.get_result_async()
    assert wait_for_future(result_future, timeout_sec=2.0)

    action_result = result_future.result()
    assert action_result.result.success
