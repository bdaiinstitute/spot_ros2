import argparse
import sys
import time

import bosdyn.client
import bosdyn.client.util
import rclpy
import rclpy.node

from bosdyn.api import image_pb2
from bosdyn.client.image import ImageClient, build_image_request
from cv_bridge import CvBridge
from google.protobuf.timestamp_pb2 import Timestamp
from sensor_msgs.msg import CameraInfo, Image

def timestamp_to_seconds(timestamp):
    return timestamp.sec + timestamp.nanosec * 1e-9

class MultipleCameraSubscriber(rclpy.node.Node):

    def __init__(self, pub_node: str, window: int):
        super().__init__(pub_node + "_subscriber")
        image_topics = self.get_publisher_names_and_types_by_node(pub_node, node_namespace="/", no_demangle=False)
        self.sub_topics = []
        for topic, _ in image_topics:
            if topic.endswith("camera_info"):
                self.subscription = self.create_subscription(
                    CameraInfo,
                    topic,
                    self.listener_callback,
                    10
                )
        self.subscription  # prevent unused variable warning
        self.aggregate_data = {}
        self.window = window


    def listener_callback(self, cam_info):
        if cam_info.header.frame_id not in self.aggregate_data:
            self.aggregate_data[cam_info.header.frame_id] = []
        agg_list = self.aggregate_data[cam_info.header.frame_id]
        if len(agg_list) >= self.window:
            start_time = min(agg_list)
            end_time = max(agg_list)
            print(f"Freq of {cam_info.header.frame_id}: {len(agg_list) / (end_time - start_time)}")
            self.aggregate_data[cam_info.header.frame_id] = []
        else:
            agg_list.append(timestamp_to_seconds(cam_info.header.stamp))

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--pub_node", type=str)
    parser.add_argument("--window_size", type=int)
    arg = parser.parse_args()
    rclpy.init(args=args)

    multiple_subscriber = MultipleCameraSubscriber(
        arg.pub_node,
        arg.window_size
    )

    rclpy.spin(multiple_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    multiple_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()