#!/usr/bin/env python3

import rclpy
import copy
import transforms3d
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import Odometry

"""
The Spot ROS 2 driver reports the Twist part of the odometry in odom frame.
According to ROS standards this should be reported in body frame.
This script subscribes to the odometry, transforms it to body frame and publishes it 
to /odometry_corrected.
"""


class OdometryTransformer(Node):

    def __init__(self):
        super().__init__('odometry_transformer')

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odometry_corrected',
            10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry',
            self.odometry_callback,
            10)
        self.odom_sub  # prevent unused variable warning

    def odometry_callback(self, msg):
        transformed_odom = self.transform_odom(msg)

        self.odom_pub.publish(transformed_odom)

    def transform_odom(self, base_odometry: Odometry):
        """
        Copied from the ROS 1 fix:
        https://github.com/heuristicus/spot_ros/blob/2a1d26da976a6376b21b5af9ce5328899fb7a8c4/spot_driver/src/spot_driver/ros_helpers.py#L402
        """

        # Current inverse rotation of body frame w.r.t. odom frame
        inverse_rotation = transforms3d.quaternions.quat2mat(
            transforms3d.quaternions.qinverse(
                [
                    base_odometry.pose.pose.orientation.w,
                    base_odometry.pose.pose.orientation.x,
                    base_odometry.pose.pose.orientation.y,
                    base_odometry.pose.pose.orientation.z,
                ]
            )
        )

        # Transform the linear twist by rotating the vector according to the rotation from body to odom
        linear_twist = np.array(
            [
                [base_odometry.twist.twist.linear.x],
                [base_odometry.twist.twist.linear.y],
                [base_odometry.twist.twist.linear.z],
            ]
        )

        corrected_linear = inverse_rotation.dot(linear_twist)

        # Do the same for the angular twist
        angular_twist = np.array(
            [
                [base_odometry.twist.twist.angular.x],
                [base_odometry.twist.twist.angular.y],
                [base_odometry.twist.twist.angular.z],
            ]
        )

        corrected_angular = inverse_rotation.dot(angular_twist)

        corrected_odometry = copy.deepcopy(base_odometry)
        corrected_odometry.twist.twist.linear.x = corrected_linear[0][0]
        corrected_odometry.twist.twist.linear.y = corrected_linear[1][0]
        corrected_odometry.twist.twist.linear.z = corrected_linear[2][0]
        corrected_odometry.twist.twist.angular.x = corrected_angular[0][0]
        corrected_odometry.twist.twist.angular.y = corrected_angular[1][0]
        corrected_odometry.twist.twist.angular.z = corrected_angular[2][0]

        return corrected_odometry


def main(args=None):
    rclpy.init(args=args)

    odometry_transformer = OdometryTransformer()

    rclpy.spin(odometry_transformer)

    odometry_transformer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()