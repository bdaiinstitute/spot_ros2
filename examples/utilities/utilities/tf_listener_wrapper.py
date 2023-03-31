# Copyright [2023] Boston Dynamics AI Institute, Inc.

import time

from bosdyn.client.math_helpers import Quat, SE3Pose, SE2Pose

from geometry_msgs.msg import Transform

import rclpy.duration
from rclpy.node import Node
from tf2_ros import ConnectivityException, ExtrapolationException, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster


def ros_transform_to_se3_pose(transform: Transform) -> SE3Pose:
    return SE3Pose(transform.translation.x, transform.translation.y, transform.translation.z,
                   Quat(w=transform.rotation.w, x=transform.rotation.x, y=transform.rotation.y, z=transform.rotation.z))


class TFListenerWrapper(object):

    def __init__(self, node_name: str, wait_for_transform: tuple[str, str] = None, cache_time_s: int = None):
        self._node = Node(node_name)  # private because we want to make sure no one but us spins this node!
        if cache_time_s is not None:
            cache_time_py = rclpy.duration.Duration(seconds=cache_time_s)
        else:
            cache_time_py = None
        self._tf_buffer = Buffer(cache_time=cache_time_py)
        # By putting in spin_thread here, the transform listener spins the node
        self._tf_listener = TransformListener(self._tf_buffer, self._node, spin_thread=True)
        if wait_for_transform is not None and len(wait_for_transform) == 2:
            self.wait_for_init(wait_for_transform[0], wait_for_transform[1])

    def shutdown(self):
        '''
        You must call this to have the program exit smoothly unfortunately.  No del command seems to work.
        '''
        self._tf_listener.executor.shutdown()
        self._tf_listener.dedicated_listener_thread.join()

    @property
    def buffer(self) -> Buffer:
        return self._tf_buffer

    def wait_for_init(self, from_frame: str, to_frame: str):
        '''
        Waits for transform from from_frame to to_frame to become available and prints initializing statements.
        '''
        # Wait for the buffer to fill to avoid annoying warnings.
        self._node.get_logger().info('Waiting for TF to contain transform from ' + str(from_frame) + ' to ' +
                                     str(to_frame))
        self.wait_for_transform(from_frame, to_frame)
        self._node.get_logger().info('TF initialized')

    def wait_for_transform(self, from_frame: str, to_frame: str):
        '''
        Wait for the transform from from to to to become available.
        '''
        # We need a while instead of the usual wait because we use a different node and a thread for our listener
        while rclpy.ok():
            try:
                self.lookup_a_tform_b(from_frame, to_frame)
                break
            except TransformException as e:
                time.sleep(0.1)

    def lookup_a_tform_b(self,
                         frame_a: str,
                         frame_b: str,
                         transform_time: float = None,
                         timeout: float = None,
                         wait_for_frames: bool = False) -> SE3Pose:
        '''
        Parameters:
            frame_a: Base frame for transform.  The transform returned will be frame_a_t_frame_b
            frame_b: Tip frame for transform.  The transform returned will be frame_a_t_frame_b
            transform_time: The time at which to look up the transform.  If left at None, will be the most recent
                transform available
            timeout: The time to wait for the transform to become available if the transform_time is beyond the most
                recent transform in the buffer.
            wait_for_frames: If true, waits timeout amount of time for a path to exist from frame_a to frame_b in the
                the buffer.  If false, this will return immediately if a path does not exist even if timeout is not
                None.  Note that wait_for_transform can be used to wait indefinitely for a transform to become
                available.
        Returns:
            The transform frame_a_t_frame_b at the time specified.
        Raises:
            All the possible TransformExceptions.
        '''
        if transform_time is None:
            transform_time = rclpy.time.Time()
        if timeout is None or not wait_for_frames:
            timeout_py = rclpy.time.Duration()
        else:
            timeout_py = rclpy.time.Duration(seconds=timeout)
        start_time = time.time()
        while rclpy.ok():
            try:
                return ros_transform_to_se3_pose(
                    self._tf_buffer.lookup_transform(frame_a, frame_b, time=transform_time,
                                                     timeout=timeout_py).transform)
            except ExtrapolationException as e:
                if 'future' not in str(e) or timeout is None:
                    raise e  # Waiting won't help with this
                now = time.time()
                if now - start_time > timeout:
                    raise e
                time.sleep(0.01)
