import contextlib
import unittest

import bdai_ros2_wrappers.scope as ros_scope
import rclpy
from std_srvs.srv import Trigger

import spot_driver.spot_ros2
from spot_msgs.srv import (  # type: ignore
    Dock,
)


class SpotDriverTest(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = contextlib.ExitStack()
        self.ros = self.fixture.enter_context(ros_scope.top(namespace="fixture"))
        # create and run spot ros2 servers
        mock_param = rclpy.parameter.Parameter("spot_name", rclpy.Parameter.Type.STRING, "Mock_spot")
        self.spot_ros2 = self.ros.load(spot_driver.spot_ros2.SpotROS, parameter_list=[mock_param])

        # clients
        self.claim_client: rclpy.node.Client = self.ros.node.create_client(Trigger, "claim")
        self.release_client: rclpy.node.Client = self.ros.node.create_client(Trigger, "release")
        self.power_on_client: rclpy.node.Client = self.ros.node.create_client(Trigger, "power_on")
        self.power_off_client: rclpy.node.Client = self.ros.node.create_client(Trigger, "power_off")
        self.sit_client: rclpy.node.Client = self.ros.node.create_client(Trigger, "sit")
        self.stand_client: rclpy.node.Client = self.ros.node.create_client(Trigger, "stand")
        self.estop_gentle: rclpy.node.Client = self.ros.node.create_client(Trigger, "estop/gentle")
        self.estop_hard: rclpy.node.Client = self.ros.node.create_client(Trigger, "estop/hard")
        self.estop_release: rclpy.node.Client = self.ros.node.create_client(Trigger, "estop/release")
        self.undock_client: rclpy.node.Client = self.ros.node.create_client(Trigger, "undock")
        self.dock_client: rclpy.node.Client = self.ros.node.create_client(Dock, "dock")

    def tearDown(self) -> None:
        self.fixture.close()

    def test_wrapped_commands(self) -> None:
        """
        Spot Ros2 driver has multiple commands that are wrapped in a service wrapper.
        When no spot_wrapper is present they return true, but this test at least tests
        communications and APIs.
        """
        resp = self.claim_client.call(Trigger.Request())
        self.assertEqual(resp.success, True)
        resp = self.release_client.call(Trigger.Request())
        self.assertEqual(resp.success, True)

        resp = self.power_on_client.call(Trigger.Request())
        self.assertEqual(resp.success, True)
        resp = self.power_off_client.call(Trigger.Request())
        self.assertEqual(resp.success, True)
        resp = self.sit_client.call(Trigger.Request())
        self.assertEqual(resp.success, True)
        resp = self.stand_client.call(Trigger.Request())
        self.assertEqual(resp.success, True)
        resp = self.estop_hard.call(Trigger.Request())
        self.assertEqual(resp.success, True)
        resp = self.estop_gentle.call(Trigger.Request())
        self.assertEqual(resp.success, True)
        resp = self.estop_release.call(Trigger.Request())
        self.assertEqual(resp.success, True)
        resp = self.undock_client.call(Trigger.Request())
        self.assertEqual(resp.success, True)
        resp = self.dock_client.call(Dock.Request())
        self.assertEqual(resp.success, True)


if __name__ == "__main__":
    unittest.main()
