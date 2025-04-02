import tempfile
from typing import Iterator

import domain_coordinator
import launch_pytest
import pytest
import synchros2.scope as ros_scope
import yaml
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_pytest.actions import ReadyToTest
from launch_ros.substitutions import FindPackageShare
from synchros2.launch.actions import update_sigterm_sigkill_timeout
from synchros2.scope import ROSAwareScope

import spot_wrapper.testing
from spot_wrapper.testing.fixtures import SpotFixture
from spot_wrapper.testing.mocks import MockSpot


# this mocks a spot with an arm
@spot_wrapper.testing.fixture
class simple_spot(MockSpot):
    def __init__(self):
        super().__init__()
        # force an entry in manipulator state to indicate robot has arm
        manipulator_state = self.robot_state.manipulator_state
        manipulator_state.is_gripper_holding_item = False


@pytest.fixture
def domain_id() -> Iterator[int]:
    with domain_coordinator.domain_id() as domain_id:  # to ensure node isolation
        yield domain_id


@pytest.fixture
def ros(simple_spot: SpotFixture, domain_id: int) -> Iterator[ROSAwareScope]:
    """
    This method is a generator function that returns a different ROS2 scope
    each time it is invoked, to handle a ROS2 context lifecycle.
    """
    with ros_scope.top(global_=True, namespace=simple_spot.api.name, domain_id=domain_id) as top:
        yield top


@launch_pytest.fixture
def robot_spot_ros2_control(simple_spot: SpotFixture, domain_id: int) -> Iterator[LaunchDescription]:
    with tempfile.NamedTemporaryFile(mode="w", suffix="config.yaml") as temp:
        data = {
            "username": "user",
            "password": "pass",
            "hostname": simple_spot.address,
            "port": simple_spot.port,
            "certificate": str(simple_spot.certificate_path),
            "spot_name": simple_spot.api.name,
        }
        yaml.dump({"/**": {"ros__parameters": data}}, temp.file)
        temp.file.close()

        ld = LaunchDescription(
            [
                SetEnvironmentVariable("ROS_DOMAIN_ID", str(domain_id)),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [
                                FindPackageShare("spot_ros2_control"),
                                "launch",
                                "spot_ros2_control.launch.py",
                            ]
                        )
                    ),
                    launch_arguments=[
                        ("config_file", temp.file.name),
                        ("hardware_interface", "robot"),
                        ("launch_image_publishers", "false"),
                        ("control_only", "true"),
                        ("spot_name", simple_spot.api.name),
                        ("launch_rviz", "False"),
                    ],
                ),
                ReadyToTest(),
            ],
        )
        update_sigterm_sigkill_timeout(ld, sigterm_timeout_s=10.0, sigkill_timeout_s=10.0)

        yield ld


def pytest_configure() -> None:
    pytest.robot_spot_ros2_control = robot_spot_ros2_control
