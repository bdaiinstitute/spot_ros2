import pytest
from synchros2.scope import ROSAwareScope

from spot_wrapper.testing.fixtures import SpotFixture


@pytest.mark.launch(fixture=pytest.spot_ros2_control_graph_description)
def test_spot_ros2_control(simple_spot: SpotFixture, ros: ROSAwareScope) -> None:
    spot_name = simple_spot.api.name
    print("spot name ", spot_name)
