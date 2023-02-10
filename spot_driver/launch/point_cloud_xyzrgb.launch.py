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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    spot_name = LaunchConfiguration('spot_name')
    spot_name_arg = DeclareLaunchArgument('spot_name', description='Name of spot')

    return LaunchDescription([
        spot_name_arg,

        # launch plugin through rclcpp_components container
        launch_ros.actions.ComposableNodeContainer(
            name='frontleft_container',
            namespace=spot_name,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_frontleft',
                    remappings=[
                        ('image_rect', PathJoinSubstitution([spot_name, "depth/frontleft/image"])),
                        ('camera_info', PathJoinSubstitution([spot_name, "depth/frontleft/camera_info"])),
                        ('image', PathJoinSubstitution([spot_name, "depth/frontleft/image"])),
                        ('points', PathJoinSubstitution([spot_name, "depth/frontleft/points"]))
                    ]
                ),
            ],
            output='screen',
        ),
        
        launch_ros.actions.ComposableNodeContainer(
            name='frontright_container',
            namespace=spot_name,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_frontright',
                    remappings=[
                        ('image_rect', PathJoinSubstitution([spot_name, "depth/frontright/image"])),
                        ('camera_info', PathJoinSubstitution([spot_name, "depth/frontright/camera_info"])),
                        ('image', PathJoinSubstitution([spot_name, "depth/frontright/image"])),
                        ('points', PathJoinSubstitution([spot_name, "depth/frontright/points"]))
                    ]
                ),
            ],
            output='screen',
        ),

        launch_ros.actions.ComposableNodeContainer(
            name='back_container',
            namespace=spot_name,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='point_cloud_xyzrgb_node',
                    remappings=[('rgb/camera_info', '/camera/color/camera_info'),
                                ('rgb/image_rect_color', '/camera/color/image_raw'),
                                ('depth_registered/image_rect', '/camera/aligned_depth_to_color/image_raw'),
                                ('points', '/camera/depth_registered/points')]
                ),
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_back',
                    remappings=[
                        ('rgb/camera_info', PathJoinSubstitution([spot_name, "depth/back/image"])),
                        ('rgb/image_rect_color', PathJoinSubstitution([spot_name, "depth/back/camera_info"])),
                        ('depth_registered/image_rect', PathJoinSubstitution([spot_name, "depth/back/image"])),
                        ('points', PathJoinSubstitution([spot_name, "depth/back/points"]))
                    ]
                ),
            ],
            output='screen',
        ),
        
        launch_ros.actions.ComposableNodeContainer(
            name='left_container',
            namespace=spot_name,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_left',
                    remappings=[
                        ('image_rect', PathJoinSubstitution([spot_name, "depth/left/image"])),
                        ('camera_info', PathJoinSubstitution([spot_name, "depth/left/camera_info"])),
                        ('image', PathJoinSubstitution([spot_name, "depth/left/image"])),
                        ('points', PathJoinSubstitution([spot_name, "depth/left/points"]))
                    ]
                ),
            ],
            output='screen',
        ),

        launch_ros.actions.ComposableNodeContainer(
            name='right_container',
            namespace=spot_name,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_right',
                    remappings=[
                        ('image_rect', PathJoinSubstitution([spot_name, "depth/right/image"])),
                        ('camera_info', PathJoinSubstitution([spot_name, "depth/right/camera_info"])),
                        ('image', PathJoinSubstitution([spot_name, "depth/right/image"])),
                        ('points', PathJoinSubstitution([spot_name, "depth/right/points"]))
                    ]
                ),
            ],
            output='screen',
        ),
    ])
