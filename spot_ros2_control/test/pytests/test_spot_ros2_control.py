import itertools

import pytest
from bosdyn.api.robot_command_pb2 import RobotCommandResponse
from bosdyn.api.robot_state_pb2 import RobotStateStreamResponse
from sensor_msgs.msg import JointState
from synchros2.futures import unwrap_future
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

NUM_JOINTS_ARM = 19


def state_stream_response(position: list[float], velocity: list[float], load: list[float]) -> RobotStateStreamResponse:
    response = RobotStateStreamResponse()
    response.joint_states.position.extend(position)
    response.joint_states.velocity.extend(velocity)
    response.joint_states.load.extend(load)
    response.inertial_state.packets.add()  # hardware interface expects at least a packet
    response.contact_states.extend([0] * 4)  # hardware interface expects all 4 feet to have a contact state
    return response


@pytest.mark.launch(fixture=pytest.robot_spot_ros2_control)
def test_joint_states(simple_spot: SpotFixture, ros: ROSAwareScope) -> None:
    # Create a state stream response containing unique joint state values for each joint
    position = list(range(0, NUM_JOINTS_ARM))
    velocity = list(range(NUM_JOINTS_ARM, 2 * NUM_JOINTS_ARM))
    load = list(range(2 * NUM_JOINTS_ARM, 3 * NUM_JOINTS_ARM))
    stream_response = state_stream_response(position=position, velocity=velocity, load=load)
    # send this response to the mock robot
    simple_spot.api.GetRobotStateStream.future.returns(itertools.repeat(stream_response))
    response = RobotCommandResponse()
    response.status = RobotCommandResponse.Status.STATUS_OK
    simple_spot.api.RobotCommand.future.returns(response)
    # start the mock joint control stream
    call = simple_spot.api.JointControlStream.serve(timeout=2.0)
    assert call is not None

    # Grab the latest joint states message from the joint state broadcaster
    spot_name = simple_spot.api.name
    joint_states = Subscription(JointState, f"/{spot_name}/low_level/joint_states", node=ros.node)
    joint_state_message = unwrap_future(joint_states.update, timeout_sec=10.0)
    expected_joints = [f"{spot_name}/{joint}" for joint in LEG_JOINTS + ARM_JOINTS]
    # ensure that the joint state message contains the values from our state stream response
    for i, joint in enumerate(expected_joints):
        assert joint in joint_state_message.name
        joint_index = joint_state_message.name.index(joint)
        assert joint_state_message.position[joint_index] == position[i]
        assert joint_state_message.velocity[joint_index] == velocity[i]
        assert joint_state_message.effort[joint_index] == load[i]

    # mock the response for the SafePowerOff command used when the hardware interface deactivates
    response = RobotCommandResponse()
    response.status = RobotCommandResponse.Status.STATUS_OK
    simple_spot.api.RobotCommand.future.returns(response)
