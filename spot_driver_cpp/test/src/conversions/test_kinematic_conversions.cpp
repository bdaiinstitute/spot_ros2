// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gtest/gtest.h>
#include <spot_driver_cpp/conversions/kinematic_conversions.hpp>

#include <rclcpp/rclcpp.hpp>

namespace spot_ros2 {

TEST(TestKinematicConversions, convert_bosdyn_msgs_inverse_kinematics_request_to_proto) {
  bosdyn_msgs::msg::InverseKinematicsRequest ros_msg;
  bosdyn::api::spot::InverseKinematicsRequest proto_msg;

  ros_msg.header.client_name = "client_name";
  ros_msg.header_is_set = true;
  ros_msg.header.request_timestamp.sec = 2;
  ros_msg.header.request_timestamp.nanosec = 200;
  ros_msg.header.request_timestamp_is_set = true;
  ros_msg.header.disable_rpc_logging = false;
  ros_msg.root_frame_name = "root_frame_name";
  ros_msg.root_tform_scene_is_set = true;
  ros_msg.root_tform_scene.position.x = 0.1;
  ros_msg.root_tform_scene.position.y = 0.2;
  ros_msg.root_tform_scene.position.z = 0.3;
  ros_msg.root_tform_scene.orientation.w = 0.7987;
  ros_msg.root_tform_scene.orientation.x = 0.1912;
  ros_msg.root_tform_scene.orientation.y = 0.4336;
  ros_msg.root_tform_scene.orientation.z = 0.3709;
  ros_msg.scene_tform_task_is_set = true;
  ros_msg.scene_tform_task.position.x = 0.1;
  ros_msg.scene_tform_task.position.y = 0.2;
  ros_msg.scene_tform_task.position.z = 0.3;
  ros_msg.scene_tform_task.orientation.w = 0.7987;
  ros_msg.scene_tform_task.orientation.x = 0.1912;
  ros_msg.scene_tform_task.orientation.y = 0.4336;
  ros_msg.scene_tform_task.orientation.z = 0.3709;
  ros_msg.nominal_arm_configuration.value =
      bosdyn_msgs::msg::InverseKinematicsRequestNamedArmConfiguration::ARM_CONFIG_READY;

  ros_msg.nominal_arm_configuration_overrides_is_set = true;
  ros_msg.nominal_arm_configuration_overrides.sh0_is_set = true;
  ros_msg.nominal_arm_configuration_overrides.sh0 = 0.1;
  ros_msg.nominal_arm_configuration_overrides.sh1_is_set = true;
  ros_msg.nominal_arm_configuration_overrides.sh1 = 0.2;
  ros_msg.nominal_arm_configuration_overrides.el0_is_set = true;
  ros_msg.nominal_arm_configuration_overrides.el0 = 0.3;
  ros_msg.nominal_arm_configuration_overrides.el1_is_set = true;
  ros_msg.nominal_arm_configuration_overrides.el1 = 0.4;
  ros_msg.nominal_arm_configuration_overrides.wr0_is_set = true;
  ros_msg.nominal_arm_configuration_overrides.wr0 = 0.5;
  ros_msg.nominal_arm_configuration_overrides.wr1_is_set = true;
  ros_msg.nominal_arm_configuration_overrides.wr1 = 0.6;

  ros_msg.scene_tform_body_nominal_is_set = true;

  kinematic_conversions::convert_bosdyn_msgs_inverse_kinematics_request_to_proto(ros_msg, proto_msg);

  ASSERT_EQ(ros_msg.header.client_name, proto_msg.header().client_name());
  ASSERT_EQ(ros_msg.header_is_set, proto_msg.has_header());
  ASSERT_EQ(ros_msg.header.request_timestamp.sec, proto_msg.header().request_timestamp().seconds());
  ASSERT_EQ(ros_msg.header.request_timestamp.nanosec, proto_msg.header().request_timestamp().nanos());
  ASSERT_EQ(ros_msg.header.request_timestamp_is_set, proto_msg.header().has_request_timestamp());
  ASSERT_EQ(ros_msg.header.disable_rpc_logging, proto_msg.header().disable_rpc_logging());
  ASSERT_EQ(ros_msg.root_frame_name, proto_msg.root_frame_name());
  ASSERT_EQ(ros_msg.root_tform_scene_is_set, proto_msg.has_root_tform_scene());
  ASSERT_EQ(ros_msg.root_tform_scene.position.x, proto_msg.root_tform_scene().position().x());
  ASSERT_EQ(ros_msg.root_tform_scene.position.y, proto_msg.root_tform_scene().position().y());
  ASSERT_EQ(ros_msg.root_tform_scene.position.z, proto_msg.root_tform_scene().position().z());
  ASSERT_EQ(ros_msg.root_tform_scene.orientation.w, proto_msg.root_tform_scene().rotation().w());
  ASSERT_EQ(ros_msg.root_tform_scene.orientation.x, proto_msg.root_tform_scene().rotation().x());
  ASSERT_EQ(ros_msg.root_tform_scene.orientation.y, proto_msg.root_tform_scene().rotation().y());
  ASSERT_EQ(ros_msg.root_tform_scene.orientation.z, proto_msg.root_tform_scene().rotation().z());
  ASSERT_EQ(ros_msg.scene_tform_task_is_set, proto_msg.has_scene_tform_task());
  ASSERT_EQ(ros_msg.scene_tform_task.position.x, proto_msg.scene_tform_task().position().x());
  ASSERT_EQ(ros_msg.scene_tform_task.position.y, proto_msg.scene_tform_task().position().y());
  ASSERT_EQ(ros_msg.scene_tform_task.position.z, proto_msg.scene_tform_task().position().z());
  ASSERT_EQ(ros_msg.scene_tform_task.orientation.w, proto_msg.scene_tform_task().rotation().w());
  ASSERT_EQ(ros_msg.scene_tform_task.orientation.x, proto_msg.scene_tform_task().rotation().x());
  ASSERT_EQ(ros_msg.scene_tform_task.orientation.y, proto_msg.scene_tform_task().rotation().y());
  ASSERT_EQ(ros_msg.scene_tform_task.orientation.z, proto_msg.scene_tform_task().rotation().z());

  // ASSERT_EQ(ros_msg.nominal_arm_configuration.value, 0);

  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides_is_set, proto_msg.has_nominal_arm_configuration_overrides());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.sh0_is_set,
            proto_msg.nominal_arm_configuration_overrides().has_sh0());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.sh0, proto_msg.nominal_arm_configuration_overrides().sh0());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.sh1_is_set,
            proto_msg.nominal_arm_configuration_overrides().has_sh1());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.sh1, proto_msg.nominal_arm_configuration_overrides().sh1());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.el0_is_set,
            proto_msg.nominal_arm_configuration_overrides().has_el0());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.el0, proto_msg.nominal_arm_configuration_overrides().el0());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.el1_is_set,
            proto_msg.nominal_arm_configuration_overrides().has_el1());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.el1, proto_msg.nominal_arm_configuration_overrides().el1());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.wr0_is_set,
            proto_msg.nominal_arm_configuration_overrides().has_wr0());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.wr0, proto_msg.nominal_arm_configuration_overrides().wr0());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.wr1_is_set,
            proto_msg.nominal_arm_configuration_overrides().has_wr1());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.wr1, proto_msg.nominal_arm_configuration_overrides().wr1());
}

}  // namespace spot_ros2