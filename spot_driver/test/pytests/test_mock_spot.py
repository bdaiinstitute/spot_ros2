
import typing

import pytest
from sensor_msgs.msg import JointState
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope
from synchros2.subscription import Subscription
from synchros2.utilities import namespace_with

from spot_wrapper.testing.fixtures import SpotFixture
import time
import os

@pytest.mark.launch(fixture=pytest.spot_graph_description)
def test_joint_states(simple_spot: SpotFixture, ros: ROSAwareScope) -> None:
    """Asserts that robot joint states are exposed over the joint_states topic."""
    kinematic_state = simple_spot.api.robot_state.kinematic_state
    for name, position in (("arm0.sh0", 0.1), ("arm0.sh1", -0.1)):
        joint_state = kinematic_state.joint_states.add()
        joint_state.name = name
        joint_state.position.value = position

    joint_states = Subscription(JointState, "joint_states", node=ros.node)
    
    start_time = time.perf_counter()
    os.system("ros2 topic list -v")
    assert wait_for_future(joint_states.update, timeout_sec=500.0)
    message = typing.cast(JointState, joint_states.latest)
    end_time = time.perf_counter()
    elapsed_time = end_time - start_time

    print(f"elapsed time: {elapsed_time} @@@@@@@@@@@@@@@@@@@@@")

    
