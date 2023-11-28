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

  // convert_bosdyn_msgs_request_header_to_proto
  ros_msg.header.request_timestamp.sec = 2;
  ros_msg.header.request_timestamp.nanosec = 200;
  ros_msg.header.request_timestamp_is_set = true;
  ros_msg.header.disable_rpc_logging = false;

  ros_msg.root_frame_name = "root_frame_name";
  ros_msg.root_tform_scene_is_set = true;

  // convert_geometry_msgs_pose_to_proto
  ros_msg.root_tform_scene.position.x = 0.1;
  ros_msg.root_tform_scene.position.y = 0.2;
  ros_msg.root_tform_scene.position.z = 0.3;
  ros_msg.root_tform_scene.orientation.w = 0.7987;
  ros_msg.root_tform_scene.orientation.x = 0.1912;
  ros_msg.root_tform_scene.orientation.y = 0.4336;
  ros_msg.root_tform_scene.orientation.z = 0.3709;

  ros_msg.scene_tform_task_is_set = true;

  // convert_geometry_msgs_pose_to_proto
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

  // convert_bosdyn_msgs_arm_joint_position_to_proto
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

  // convert_geometry_msgs_pose_to_proto
  ros_msg.scene_tform_body_nominal_is_set = true;
  ros_msg.scene_tform_body_nominal.position.x = 0.1;
  ros_msg.scene_tform_body_nominal.position.y = 0.2;
  ros_msg.scene_tform_body_nominal.position.z = 0.3;
  ros_msg.scene_tform_body_nominal.orientation.w = 0.7987;
  ros_msg.scene_tform_body_nominal.orientation.x = 0.1912;
  ros_msg.scene_tform_body_nominal.orientation.y = 0.4336;
  ros_msg.scene_tform_body_nominal.orientation.z = 0.3709;

  // convert_bosdyn_msgs_inverse_kinematics_request_one_of_stance_specification_to_proto
  ros_msg.stance_specification.stance_specification_choice =
      bosdyn_msgs::msg::InverseKinematicsRequestOneOfStanceSpecification::STANCE_SPECIFICATION_NOT_SET;

  // convert_bosdyn_msgs_inverse_kinematics_request_one_of_tool_specification_to_proto

  // convert_bosdyn_msgs_inverse_kinematics_request_one_of_task_specification_to_proto

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
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.sh0,
            proto_msg.nominal_arm_configuration_overrides().sh0().value());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.sh1_is_set,
            proto_msg.nominal_arm_configuration_overrides().has_sh1());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.sh1,
            proto_msg.nominal_arm_configuration_overrides().sh1().value());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.el0_is_set,
            proto_msg.nominal_arm_configuration_overrides().has_el0());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.el0,
            proto_msg.nominal_arm_configuration_overrides().el0().value());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.el1_is_set,
            proto_msg.nominal_arm_configuration_overrides().has_el1());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.el1,
            proto_msg.nominal_arm_configuration_overrides().el1().value());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.wr0_is_set,
            proto_msg.nominal_arm_configuration_overrides().has_wr0());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.wr0,
            proto_msg.nominal_arm_configuration_overrides().wr0().value());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.wr1_is_set,
            proto_msg.nominal_arm_configuration_overrides().has_wr1());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides.wr1,
            proto_msg.nominal_arm_configuration_overrides().wr1().value());

  //   ASSERT_EQ(ros_msg.scene_tform_body_nominal_is_set, proto_msg.has_scene_tform_body_nominal());
  //   ASSERT_EQ(ros_msg.scene_tform_body_nominal.position.x, proto_msg.scene_tform_body_nominal().position().x());
  //   ASSERT_EQ(ros_msg.scene_tform_body_nominal.position.y, proto_msg.scene_tform_body_nominal().position().y());
  //   ASSERT_EQ(ros_msg.scene_tform_body_nominal.position.z, proto_msg.scene_tform_body_nominal().position().z());
  //   ASSERT_EQ(ros_msg.scene_tform_body_nominal.orientation.w,
  //   proto_msg.scene_tform_body_nominal().orientation().w());
  //   ASSERT_EQ(ros_msg.scene_tform_body_nominal.orientation.x,
  //   proto_msg.scene_tform_body_nominal().orientation().x());
  //   ASSERT_EQ(ros_msg.scene_tform_body_nominal.orientation.y,
  //   proto_msg.scene_tform_body_nominal().orientation().y());
  //   ASSERT_EQ(ros_msg.scene_tform_body_nominal.orientation.z,
  //   proto_msg.scene_tform_body_nominal().orientation().z());

  ASSERT_EQ(ros_msg.stance_specification.stance_specification_choice, proto_msg.stance_specification_case());
}

TEST(TestKinematicConversions, convert_bosdyn_msgs_inverse_kinematics_request_fixed_stance_to_proto) {
  bosdyn_msgs::msg::InverseKinematicsRequest ros_msg;
  bosdyn::api::spot::InverseKinematicsRequest proto_msg;

  ros_msg.stance_specification.fixed_stance.fl_rt_scene_is_set = true;
  ros_msg.stance_specification.fixed_stance.fl_rt_scene.x = 0.1;
  ros_msg.stance_specification.fixed_stance.fl_rt_scene.y = 0.2;
  ros_msg.stance_specification.fixed_stance.fl_rt_scene.z = 0.3;

  ros_msg.stance_specification.fixed_stance.fr_rt_scene_is_set = true;
  ros_msg.stance_specification.fixed_stance.fr_rt_scene.x = 0.1;
  ros_msg.stance_specification.fixed_stance.fr_rt_scene.y = 0.2;
  ros_msg.stance_specification.fixed_stance.fr_rt_scene.z = 0.3;

  ros_msg.stance_specification.fixed_stance.hl_rt_scene_is_set = true;
  ros_msg.stance_specification.fixed_stance.hl_rt_scene.x = 0.1;
  ros_msg.stance_specification.fixed_stance.hl_rt_scene.y = 0.2;
  ros_msg.stance_specification.fixed_stance.hl_rt_scene.z = 0.3;

  ros_msg.stance_specification.fixed_stance.hr_rt_scene_is_set = true;
  ros_msg.stance_specification.fixed_stance.hr_rt_scene.x = 0.1;
  ros_msg.stance_specification.fixed_stance.hr_rt_scene.y = 0.2;
  ros_msg.stance_specification.fixed_stance.hr_rt_scene.z = 0.3;

  kinematic_conversions::convert_bosdyn_msgs_inverse_kinematics_request_fixed_stance_to_proto(
      ros_msg.stance_specification.fixed_stance, *proto_msg.mutable_fixed_stance());

  ASSERT_EQ(ros_msg.stance_specification.fixed_stance.fl_rt_scene_is_set, proto_msg.fixed_stance().has_fl_rt_scene());
  ASSERT_EQ(ros_msg.stance_specification.fixed_stance.fl_rt_scene.x, proto_msg.fixed_stance().fl_rt_scene().x());
  ASSERT_EQ(ros_msg.stance_specification.fixed_stance.fl_rt_scene.y, proto_msg.fixed_stance().fl_rt_scene().y());
  ASSERT_EQ(ros_msg.stance_specification.fixed_stance.fl_rt_scene.z, proto_msg.fixed_stance().fl_rt_scene().z());

  ASSERT_EQ(ros_msg.stance_specification.fixed_stance.fr_rt_scene_is_set, proto_msg.fixed_stance().has_fr_rt_scene());
  ASSERT_EQ(ros_msg.stance_specification.fixed_stance.fr_rt_scene.x, proto_msg.fixed_stance().fr_rt_scene().x());
  ASSERT_EQ(ros_msg.stance_specification.fixed_stance.fr_rt_scene.y, proto_msg.fixed_stance().fr_rt_scene().y());
  ASSERT_EQ(ros_msg.stance_specification.fixed_stance.fr_rt_scene.z, proto_msg.fixed_stance().fr_rt_scene().z());

  ASSERT_EQ(ros_msg.stance_specification.fixed_stance.hl_rt_scene_is_set, proto_msg.fixed_stance().has_hl_rt_scene());
  ASSERT_EQ(ros_msg.stance_specification.fixed_stance.hl_rt_scene.x, proto_msg.fixed_stance().hl_rt_scene().x());
  ASSERT_EQ(ros_msg.stance_specification.fixed_stance.hl_rt_scene.y, proto_msg.fixed_stance().hl_rt_scene().y());
  ASSERT_EQ(ros_msg.stance_specification.fixed_stance.hl_rt_scene.z, proto_msg.fixed_stance().hl_rt_scene().z());

  ASSERT_EQ(ros_msg.stance_specification.fixed_stance.hr_rt_scene_is_set, proto_msg.fixed_stance().has_hr_rt_scene());
  ASSERT_EQ(ros_msg.stance_specification.fixed_stance.hr_rt_scene.x, proto_msg.fixed_stance().hr_rt_scene().x());
  ASSERT_EQ(ros_msg.stance_specification.fixed_stance.hr_rt_scene.y, proto_msg.fixed_stance().hr_rt_scene().y());
  ASSERT_EQ(ros_msg.stance_specification.fixed_stance.hr_rt_scene.z, proto_msg.fixed_stance().hr_rt_scene().z());
}

TEST(TestKinematicConversions, convert_bosdyn_msgs_inverse_kinematics_request_on_ground_plane_stance_to_proto) {
  bosdyn_msgs::msg::InverseKinematicsRequest ros_msg;
  bosdyn::api::spot::InverseKinematicsRequest proto_msg;

  ros_msg.stance_specification.on_ground_plane_stance.scene_tform_ground_is_set = true;
  ros_msg.stance_specification.on_ground_plane_stance.scene_tform_ground.position.x = 0.1;
  ros_msg.stance_specification.on_ground_plane_stance.scene_tform_ground.position.y = 0.2;
  ros_msg.stance_specification.on_ground_plane_stance.scene_tform_ground.position.z = 0.3;
  ros_msg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.w = 0.7987;
  ros_msg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.x = 0.1912;
  ros_msg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.y = 0.4336;
  ros_msg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.z = 0.3709;

  kinematic_conversions::convert_bosdyn_msgs_inverse_kinematics_request_on_ground_plane_stance_to_proto(
      ros_msg.stance_specification.on_ground_plane_stance, *proto_msg.mutable_on_ground_plane_stance());

  ASSERT_EQ(ros_msg.stance_specification.on_ground_plane_stance.scene_tform_ground_is_set,
            proto_msg.on_ground_plane_stance().has_scene_tform_ground());
  ASSERT_EQ(ros_msg.stance_specification.on_ground_plane_stance.scene_tform_ground.position.x,
            proto_msg.on_ground_plane_stance().scene_tform_ground().position().x());
  ASSERT_EQ(ros_msg.stance_specification.on_ground_plane_stance.scene_tform_ground.position.y,
            proto_msg.on_ground_plane_stance().scene_tform_ground().position().y());
  ASSERT_EQ(ros_msg.stance_specification.on_ground_plane_stance.scene_tform_ground.position.z,
            proto_msg.on_ground_plane_stance().scene_tform_ground().position().z());
  ASSERT_EQ(ros_msg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.w,
            proto_msg.on_ground_plane_stance().scene_tform_ground().rotation().w());
  ASSERT_EQ(ros_msg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.x,
            proto_msg.on_ground_plane_stance().scene_tform_ground().rotation().x());
  ASSERT_EQ(ros_msg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.y,
            proto_msg.on_ground_plane_stance().scene_tform_ground().rotation().y());
  ASSERT_EQ(ros_msg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.z,
            proto_msg.on_ground_plane_stance().scene_tform_ground().rotation().z());
}

}  // namespace spot_ros2