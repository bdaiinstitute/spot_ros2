import itertools

import pytest
from bosdyn.api.robot_state_pb2 import RobotStateStreamResponse
from sensor_msgs.msg import JointState
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope
from synchros2.subscription import Subscription

from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.launch(fixture=pytest.mock_spot_ros2_control)
def test_mock(ros: ROSAwareScope) -> None:
    print("Hello")
    joint_states = Subscription(JointState, "/low_level/joint_states", node=ros.node)
    assert wait_for_future(joint_states.update, timeout_sec=10.0)
    message = joint_states.latest
    print(message)


NUM_JOINTS = 19


def zero_robot_state_stream_response() -> RobotStateStreamResponse:
    zero_response = RobotStateStreamResponse()
    zero_response.joint_states.position.extend([0.0] * NUM_JOINTS)
    zero_response.joint_states.velocity.extend([0.0] * NUM_JOINTS)
    zero_response.joint_states.load.extend([0.0] * NUM_JOINTS)
    zero_response.inertial_state.packets.add()  # hardware interface expects at least a packet
    zero_response.contact_states.extend([0] * 4)  # hardware interface expects all 4 feet to have a contact state
    return zero_response


@pytest.mark.launch(fixture=pytest.robot_spot_ros2_control)
def test_spot_ros2_control(simple_spot: SpotFixture, ros: ROSAwareScope) -> None:
    simple_spot.api.GetRobotStateStream.future.returns(itertools.repeat(zero_robot_state_stream_response()))
    spot_name = simple_spot.api.name
    print("spot name ", spot_name)
    joint_states = Subscription(JointState, f"/{spot_name}/low_level/joint_states", node=ros.node)
    assert wait_for_future(joint_states.update, timeout_sec=20.0)
    message = joint_states.latest
    print(message)
