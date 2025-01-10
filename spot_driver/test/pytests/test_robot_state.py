# Copyright (c) 2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import typing

import pytest
from sensor_msgs.msg import JointState
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope
from synchros2.subscription import Subscription
from synchros2.utilities import namespace_with

from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.launch(fixture=pytest.spot_graph_description)
def test_joint_states(simple_spot: SpotFixture, ros: ROSAwareScope) -> None:
    """Asserts that robot joint states are exposed over the joint_states topic."""
    kinematic_state = simple_spot.api.robot_state.kinematic_state
    for name, position in (("arm0.sh0", 0.1), ("arm0.sh1", -0.1)):
        joint_state = kinematic_state.joint_states.add()
        joint_state.name = name
        joint_state.position.value = position

    joint_states = Subscription(JointState, "joint_states", node=ros.node)
    assert wait_for_future(joint_states.update, timeout_sec=10.0)
    message = typing.cast(JointState, joint_states.latest)

    expected_joint_positions = {
        namespace_with(simple_spot.api.name, "arm_sh0"): 0.1,
        namespace_with(simple_spot.api.name, "arm_sh1"): -0.1,
    }
    joint_positions = {name: position for name, position in zip(message.name, message.position)}
    assert expected_joint_positions == joint_positions
