import itertools

import pytest
from bosdyn.api.robot_command_pb2 import RobotCommandResponse
from bosdyn.api.robot_state_pb2 import RobotStateStreamResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope
from synchros2.subscription import Subscription

from spot_wrapper.testing.fixtures import SpotFixture

LEG_JOINTS = [
    "front_left_hip_x",
    "front_left_hip_y",
    "front_left_knee",
    "front_right_hip_x",
    "front_right_hip_y",
    "front_right_knee",
    "rear_left_hip_x",
    "rear_left_hip_y",
    "rear_left_knee",
    "rear_right_hip_x",
    "rear_right_hip_y",
    "rear_right_knee",
]
ARM_JOINTS = ["arm_sh0", "arm_sh1", "arm_el0", "arm_el1", "arm_wr0", "arm_wr1", "arm_f1x"]


@pytest.mark.launch(fixture=pytest.mock_spot_ros2_control)
def test_mock_forward_position(ros: ROSAwareScope) -> None:
    print("Hello")
    # TODO update this, we should be able to programatically wait until everything is active...
    # time.sleep(5)
    joint_states_sub = Subscription(JointState, "/low_level/joint_states", node=ros.node)
    command_pub = ros.node.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)
    assert wait_for_future(joint_states_sub.update, timeout_sec=10.0)
    # get a joint angles message before sending any command.
    initial_joint_states = joint_states_sub.latest

    expected_joints = LEG_JOINTS + ARM_JOINTS
    njoints = len(expected_joints)
    # check that the size matches the joints we expect.
    assert len(initial_joint_states.name) == njoints
    assert len(initial_joint_states.position) == njoints
    assert len(initial_joint_states.velocity) == njoints
    assert len(initial_joint_states.effort) == njoints
    for joint in expected_joints:
        assert joint in initial_joint_states.name

    # try publishing commands to forward position controller.
    # this sends a dummy command of 1 to front_left_hip_x, 2 to front_left_hip_y, ...
    command = Float64MultiArray(data=list(range(1, njoints + 1)))
    command_pub.publish(command)
    assert wait_for_future(joint_states_sub.update, timeout_sec=10.0)
    # get the joint state message after sending the command, and check that the update is reflected
    updated_joint_states = joint_states_sub.latest
    print("\n\n\n\n\n\n\n\n")
    print(initial_joint_states.position)
    print(updated_joint_states.position)
    for i, joint in enumerate(expected_joints):
        assert joint in updated_joint_states.name
        joint_index = updated_joint_states.name.index(joint)
        # position value should be updated with the command
        assert updated_joint_states.position[joint_index] == command.data[i]
        # velocity and effort should be unchanged
        assert updated_joint_states.velocity[joint_index] == initial_joint_states.velocity[joint_index]
        assert updated_joint_states.effort[joint_index] == initial_joint_states.effort[joint_index]


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
    response = RobotCommandResponse()
    response.status = RobotCommandResponse.Status.STATUS_OK
    simple_spot.api.RobotCommand.future.returns(response)
    call = simple_spot.api.JointControlStream.serve(timeout=2.0)
    assert call is not None
    spot_name = simple_spot.api.name
    joint_states = Subscription(JointState, f"/{spot_name}/low_level/joint_states", node=ros.node)
    assert wait_for_future(joint_states.update, timeout_sec=20.0)
    message = joint_states.latest
    print(message)
