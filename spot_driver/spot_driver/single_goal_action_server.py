import threading
from typing import Any, Callable, Optional, TypeVar

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import CallbackGroup, ReentrantCallbackGroup
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node

# Note: for this to work correctly you must use a multi-threaded executor when spinning the node!  E.g.:
#   from rclpy.executors import MultiThreadedExecutor
#   executor = MultiThreadedExecutor()
#   rclpy.spin(node, executor=executor)


ActionType = TypeVar("ActionType")


class SingleGoalActionServer(object):
    def __init__(
        self,
        node: Node,
        action_type: ActionType,
        action_topic: str,
        execute_callback: Callable,
        callback_group: Optional[CallbackGroup] = None,
    ) -> None:
        self._node = node
        self._goal_handle: Optional[ServerGoalHandle] = None
        self._goal_lock = threading.Lock()
        self._callback_group = callback_group
        if self._callback_group is None:
            self._callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            node,
            action_type,
            action_topic,
            execute_callback=execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )

    def get_logger(self) -> RcutilsLogger:
        return self._node.get_logger()

    def destroy(self) -> None:
        self._action_server.destroy()

    def goal_callback(self, goal_request: Any) -> GoalResponse:
        """Accept or reject a client request to begin an action."""
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info("Aborting previous goal")
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal: Any) -> CancelResponse:
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT
