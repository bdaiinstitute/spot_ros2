import unittest
from threading import Thread
from typing import Any, Optional

import rclpy
from rclpy import Context
from rclpy.callback_groups import CallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from std_srvs.srv import Trigger

import spot_driver.spot_ros2
from spot_msgs.srv import (  # type: ignore
    Dock,
)


def spin_thread(executor: MultiThreadedExecutor) -> None:
    if executor is not None:
        try:
            executor.spin()
        except (ExternalShutdownException, KeyboardInterrupt):
            pass


def call_trigger_client(
    client: rclpy.node.Client, executor: MultiThreadedExecutor, request: Any = Trigger.Request()
) -> spot_driver.spot_ros2.Response:
    req = request
    future = client.call_async(req)
    executor.spin_until_future_complete(future)
    resp = future.result()
    return resp


class SpotDriverTest(unittest.TestCase):
    def setUp(self) -> None:
        self.context: Optional[Context] = Context()
        rclpy.init()

        # create and run spot ros2 servers
        mock_param = rclpy.parameter.Parameter("spot_name", rclpy.Parameter.Type.STRING, "Mock_spot")
        self.spot_ros2 = spot_driver.spot_ros2.SpotROS(parameter_list=[mock_param])
        # set mock
        self.spot_ros2_exec = MultiThreadedExecutor(num_threads=8)
        self.spot_ros2_exec.add_node(self.spot_ros2)
        self.spot_ros2_thread: Thread = Thread(target=spin_thread, args=(self.spot_ros2_exec,))
        self.spot_ros2_thread.start()
        self.group: CallbackGroup = ReentrantCallbackGroup()

        # clients
        self.client_node = rclpy.node.Node(node_name="client_tester")
        self.command_exec: MultiThreadedExecutor = MultiThreadedExecutor(num_threads=8)
        self.command_exec.add_node(self.client_node)
        self.claim_client: rclpy.node.Client = self.client_node.create_client(
            Trigger, "claim", callback_group=self.group
        )
        self.release_client: rclpy.node.Client = self.client_node.create_client(
            Trigger, "release", callback_group=self.group
        )
        self.power_on_client: rclpy.node.Client = self.client_node.create_client(
            Trigger, "power_on", callback_group=self.group
        )
        self.power_off_client: rclpy.node.Client = self.client_node.create_client(
            Trigger, "power_off", callback_group=self.group
        )
        self.sit_client: rclpy.node.Client = self.client_node.create_client(Trigger, "sit", callback_group=self.group)
        self.stand_client: rclpy.node.Client = self.client_node.create_client(
            Trigger, "stand", callback_group=self.group
        )
        self.estop_gentle: rclpy.node.Client = self.client_node.create_client(
            Trigger, "estop/gentle", callback_group=self.group
        )
        self.estop_hard: rclpy.node.Client = self.client_node.create_client(
            Trigger, "estop/hard", callback_group=self.group
        )
        self.estop_release: rclpy.node.Client = self.client_node.create_client(
            Trigger, "estop/release", callback_group=self.group
        )
        self.undock_client: rclpy.node.Client = self.client_node.create_client(
            Trigger, "undock", callback_group=self.group
        )
        self.dock_client: rclpy.node.Client = self.client_node.create_client(Dock, "dock", callback_group=self.group)

    def tearDown(self) -> None:
        # shutdown and kill any nodes and threads
        if self.command_exec is not None:
            self.command_exec.shutdown()
            self.command_exec.remove_node(self.client_node)
            self.client_node.destroy_node()

        if self.spot_ros2_exec is not None:
            self.spot_ros2_exec.shutdown()
            self.spot_ros2_exec.remove_node(self.spot_ros2)
            self.spot_ros2_thread.join()
            self.spot_ros2.destroy_node()

        rclpy.shutdown()
        self.context = None

    def test_wrapped_commands(self) -> None:
        """
        Spot Ros2 driver has multiple commands that are wrapped in a service wrapper.
        When no spot_wrapper is present they return true, but this test at least tests
        communications and APIs.
        """
        resp = call_trigger_client(self.claim_client, self.command_exec)
        self.assertEqual(resp.success, True)
        resp = call_trigger_client(self.release_client, self.command_exec)
        self.assertEqual(resp.success, True)
        resp = call_trigger_client(self.power_on_client, self.command_exec)
        self.assertEqual(resp.success, True)
        resp = call_trigger_client(self.power_off_client, self.command_exec)
        self.assertEqual(resp.success, True)
        resp = call_trigger_client(self.sit_client, self.command_exec)
        self.assertEqual(resp.success, True)
        resp = call_trigger_client(self.stand_client, self.command_exec)
        self.assertEqual(resp.success, True)
        resp = call_trigger_client(self.estop_hard, self.command_exec)
        self.assertEqual(resp.success, True)
        resp = call_trigger_client(self.estop_gentle, self.command_exec)
        self.assertEqual(resp.success, True)
        resp = call_trigger_client(self.estop_release, self.command_exec)
        self.assertEqual(resp.success, True)
        resp = call_trigger_client(self.undock_client, self.command_exec)
        self.assertEqual(resp.success, True)
        resp = call_trigger_client(self.dock_client, self.command_exec, request=Dock.Request())
        self.assertEqual(resp.success, True)


if __name__ == "__main__":
    unittest.main()
