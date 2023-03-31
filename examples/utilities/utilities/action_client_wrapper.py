# Copyright [2023] Boston Dynamics AI Institute, Inc.

import rclpy.action
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


class ActionClientWrapper(rclpy.action.ActionClient):

    def __init__(self, action_type, action_name, namespace=None):
        self._node = Node(action_name + '_client_wrapper_node', namespace=namespace)
        super().__init__(self._node, action_type, action_name)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._node.get_logger().info('Waiting for action server for ' + self._node.get_namespace() + '/' + action_name)
        self.wait_for_server()
        self._node.get_logger().info('Found server')

    def send_goal_and_wait(self, action_goal, timeout_sec=None):
        future = self.send_goal_async(action_goal)
        self._executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().error('Goal was not accepted')
            return None
        future = goal_handle.get_result_async()
        self._executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        return future.result()
