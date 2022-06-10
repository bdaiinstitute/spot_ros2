# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    default_rviz = os.path.join(get_package_share_directory('depth_image_proc'),
                                'launch', 'rviz/point_cloud_xyz.rviz')
    spot_rviz = os.path.join(get_package_share_directory('spot_driver'),
                                'rviz', 'viz_spot.rviz')

    spot_driver = get_package_share_directory('spot_driver')

    return LaunchDescription([
        # install realsense from https://github.com/intel/ros2_intel_realsense
        # launch plugin through rclcpp_components container
        launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_frontleft',
                    remappings=[('image_rect', '/depth/frontleft/image'),
                                ('camera_info', '/depth/frontleft/camera_info'),
                                ('image', '/camera/frontleft/image'),
                                ('points', 'points_frontleft')]
                ),
            ],
            output='screen',
        ),
        
        launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_frontright',
                    remappings=[('image_rect', '/depth/frontright/image'),
                                ('camera_info', '/depth/frontright/camera_info'),
                                ('image', '/camera/frontright/image'),
                                ('points', 'points_frontright')]
                ),
            ],
            output='screen',
        ),

        launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_back',
                    remappings=[('image_rect', '/depth/back/image'),
                                ('camera_info', '/depth/back/camera_info'),
                                ('image', '/camera/back/image'),
                                ('points', 'points_back')]
                ),
            ],
            output='screen',
        ),
        
        launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_left',
                    remappings=[('image_rect', '/depth/left/image'),
                                ('camera_info', '/depth/left/camera_info'),
                                ('image', '/camera/left/image'),
                                ('points', 'points_left')]
                ),
            ],
            output='screen',
        ),

        launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_right',
                    remappings=[('image_rect', '/depth/right/image'),
                                ('camera_info', '/depth/right/camera_info'),
                                ('image', '/camera/right/image'),
                                ('points', 'points_right')]
                ),
            ],
            output='screen',
        ),

        # rviz
        # launch_ros.actions.Node(
        #     package='rviz2',
        #     namespace= 'rviz2',
        #     executable= 'rviz2',
        #     name= 'rviz2',
        #     arguments=[ '--display-config', spot_rviz]
        # ),
    ])
