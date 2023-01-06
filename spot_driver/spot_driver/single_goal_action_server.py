import threading

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup


# Note: for this to work correctly you must use a multi-threaded executor when spinning the node!  E.g.:
#   from rclpy.executors import MultiThreadedExecutor
#   executor = MultiThreadedExecutor()
#   rclpy.spin(node, executor=executor)

class SingleGoalActionServer(object):

    def __init__(self, node, action_type, action_topic, execute_callback, callback_group=None):
        self._node = node
        self._goal_handle = None
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
            callback_group=self._callback_group)

    def get_logger(self):
        return self._node.get_logger()

    def destroy(self):
        self._action_server.destroy()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
