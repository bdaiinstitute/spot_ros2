// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gtest/gtest.h>
#include <spot_driver_cpp/conversions/kinematic_conversions.hpp>

#include <rclcpp/rclcpp.hpp>

namespace spot_ros2 {

///////////////////////////////////////////////////////////////////////////////
// ROS to Protobuf.

TEST(TestKinematicConversions, convert_bosdyn_msgs_inverse_kinematics_request_to_proto) {
  bosdyn_msgs::msg::InverseKinematicsRequest ros_msg;
  bosdyn::api::spot::InverseKinematicsRequest proto_msg;

  ros_msg.header_is_set = true;
  ros_msg.root_frame_name = "root_frame_name";
  ros_msg.root_tform_scene_is_set = true;
  ros_msg.scene_tform_task_is_set = true;
  ros_msg.nominal_arm_configuration.value =
      bosdyn_msgs::msg::InverseKinematicsRequestNamedArmConfiguration::ARM_CONFIG_READY;
  ros_msg.nominal_arm_configuration_overrides_is_set = true;
  ros_msg.scene_tform_body_nominal_is_set = true;

  kinematic_conversions::convert_bosdyn_msgs_inverse_kinematics_request_to_proto(ros_msg, proto_msg);

  ASSERT_EQ(ros_msg.header_is_set, proto_msg.has_header());
  ASSERT_EQ(ros_msg.root_frame_name, proto_msg.root_frame_name());
  ASSERT_EQ(ros_msg.root_tform_scene_is_set, proto_msg.has_root_tform_scene());
  ASSERT_EQ(ros_msg.scene_tform_task_is_set, proto_msg.has_scene_tform_task());
  ASSERT_EQ(ros_msg.nominal_arm_configuration.value, proto_msg.nominal_arm_configuration());
  ASSERT_EQ(ros_msg.nominal_arm_configuration_overrides_is_set, proto_msg.has_nominal_arm_configuration_overrides());
  ASSERT_EQ(ros_msg.scene_tform_body_nominal_is_set, proto_msg.has_scene_tform_body_nominal());
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

TEST(TestKinematicConversions,
     convert_bosdyn_msgs_inverse_kinematics_request_one_of_tool_specification_to_proto_wrist_mounted) {
  bosdyn_msgs::msg::InverseKinematicsRequestOneOfToolSpecification ros_msg;
  bosdyn::api::spot::InverseKinematicsRequest proto_msg;

  ros_msg.wrist_mounted_tool.wrist_tform_tool_is_set = true;
  ros_msg.tool_specification_choice = ros_msg.TOOL_SPECIFICATION_WRIST_MOUNTED_TOOL_SET;
  ros_msg.wrist_mounted_tool.wrist_tform_tool.position.x = 0.1;
  ros_msg.wrist_mounted_tool.wrist_tform_tool.position.y = 0.2;
  ros_msg.wrist_mounted_tool.wrist_tform_tool.position.z = 0.3;
  ros_msg.wrist_mounted_tool.wrist_tform_tool.orientation.w = 0.7987;
  ros_msg.wrist_mounted_tool.wrist_tform_tool.orientation.x = 0.1912;
  ros_msg.wrist_mounted_tool.wrist_tform_tool.orientation.y = 0.4336;
  ros_msg.wrist_mounted_tool.wrist_tform_tool.orientation.z = 0.3709;

  kinematic_conversions::convert_bosdyn_msgs_inverse_kinematics_request_one_of_tool_specification_to_proto(ros_msg,
                                                                                                           proto_msg);
  ASSERT_EQ(ros_msg.wrist_mounted_tool.wrist_tform_tool.position.x,
            proto_msg.wrist_mounted_tool().wrist_tform_tool().position().x());
  ASSERT_EQ(ros_msg.wrist_mounted_tool.wrist_tform_tool.position.y,
            proto_msg.wrist_mounted_tool().wrist_tform_tool().position().y());
  ASSERT_EQ(ros_msg.wrist_mounted_tool.wrist_tform_tool.position.z,
            proto_msg.wrist_mounted_tool().wrist_tform_tool().position().z());
  ASSERT_EQ(ros_msg.wrist_mounted_tool.wrist_tform_tool.orientation.w,
            proto_msg.wrist_mounted_tool().wrist_tform_tool().rotation().w());
  ASSERT_EQ(ros_msg.wrist_mounted_tool.wrist_tform_tool.orientation.x,
            proto_msg.wrist_mounted_tool().wrist_tform_tool().rotation().x());
  ASSERT_EQ(ros_msg.wrist_mounted_tool.wrist_tform_tool.orientation.y,
            proto_msg.wrist_mounted_tool().wrist_tform_tool().rotation().y());
  ASSERT_EQ(ros_msg.wrist_mounted_tool.wrist_tform_tool.orientation.z,
            proto_msg.wrist_mounted_tool().wrist_tform_tool().rotation().z());
}

TEST(TestKinematicConversions,
     convert_bosdyn_msgs_inverse_kinematics_request_one_of_tool_specification_to_proto_body_mounted) {
  bosdyn_msgs::msg::InverseKinematicsRequestOneOfToolSpecification ros_msg;
  bosdyn::api::spot::InverseKinematicsRequest proto_msg;

  ros_msg.body_mounted_tool.body_tform_tool_is_set = true;
  ros_msg.tool_specification_choice = ros_msg.TOOL_SPECIFICATION_BODY_MOUNTED_TOOL_SET;
  ros_msg.body_mounted_tool.body_tform_tool.position.x = 0.1;
  ros_msg.body_mounted_tool.body_tform_tool.position.y = 0.2;
  ros_msg.body_mounted_tool.body_tform_tool.position.z = 0.3;
  ros_msg.body_mounted_tool.body_tform_tool.orientation.w = 0.7987;
  ros_msg.body_mounted_tool.body_tform_tool.orientation.x = 0.1912;
  ros_msg.body_mounted_tool.body_tform_tool.orientation.y = 0.4336;
  ros_msg.body_mounted_tool.body_tform_tool.orientation.z = 0.3709;

  kinematic_conversions::convert_bosdyn_msgs_inverse_kinematics_request_one_of_tool_specification_to_proto(ros_msg,
                                                                                                           proto_msg);
  ASSERT_EQ(ros_msg.body_mounted_tool.body_tform_tool.position.x,
            proto_msg.body_mounted_tool().body_tform_tool().position().x());
  ASSERT_EQ(ros_msg.body_mounted_tool.body_tform_tool.position.y,
            proto_msg.body_mounted_tool().body_tform_tool().position().y());
  ASSERT_EQ(ros_msg.body_mounted_tool.body_tform_tool.position.z,
            proto_msg.body_mounted_tool().body_tform_tool().position().z());
  ASSERT_EQ(ros_msg.body_mounted_tool.body_tform_tool.orientation.w,
            proto_msg.body_mounted_tool().body_tform_tool().rotation().w());
  ASSERT_EQ(ros_msg.body_mounted_tool.body_tform_tool.orientation.x,
            proto_msg.body_mounted_tool().body_tform_tool().rotation().x());
  ASSERT_EQ(ros_msg.body_mounted_tool.body_tform_tool.orientation.y,
            proto_msg.body_mounted_tool().body_tform_tool().rotation().y());
  ASSERT_EQ(ros_msg.body_mounted_tool.body_tform_tool.orientation.z,
            proto_msg.body_mounted_tool().body_tform_tool().rotation().z());
}

TEST(TestKinematicConversions, convert_bosdyn_msgs_inverse_kinematics_request_one_of_task_specification_to_proto_pose) {
  bosdyn_msgs::msg::InverseKinematicsRequestOneOfTaskSpecification ros_msg;
  bosdyn::api::spot::InverseKinematicsRequest proto_msg;

  ros_msg.task_specification_choice =
      bosdyn_msgs::msg::InverseKinematicsRequestOneOfTaskSpecification::TASK_SPECIFICATION_TOOL_POSE_TASK_SET;
  ros_msg.tool_pose_task.task_tform_desired_tool_is_set = true;
  ros_msg.tool_pose_task.task_tform_desired_tool.position.x = 0.1;
  ros_msg.tool_pose_task.task_tform_desired_tool.position.y = 0.2;
  ros_msg.tool_pose_task.task_tform_desired_tool.position.z = 0.3;
  ros_msg.tool_pose_task.task_tform_desired_tool.orientation.w = 0.7987;
  ros_msg.tool_pose_task.task_tform_desired_tool.orientation.x = 0.1912;
  ros_msg.tool_pose_task.task_tform_desired_tool.orientation.y = 0.4336;
  ros_msg.tool_pose_task.task_tform_desired_tool.orientation.z = 0.3709;

  kinematic_conversions::convert_bosdyn_msgs_inverse_kinematics_request_one_of_task_specification_to_proto(ros_msg,
                                                                                                           proto_msg);
  ASSERT_EQ(ros_msg.tool_pose_task.task_tform_desired_tool.position.x,
            proto_msg.tool_pose_task().task_tform_desired_tool().position().x());
  ASSERT_EQ(ros_msg.tool_pose_task.task_tform_desired_tool.position.y,
            proto_msg.tool_pose_task().task_tform_desired_tool().position().y());
  ASSERT_EQ(ros_msg.tool_pose_task.task_tform_desired_tool.position.z,
            proto_msg.tool_pose_task().task_tform_desired_tool().position().z());
  ASSERT_EQ(ros_msg.tool_pose_task.task_tform_desired_tool.orientation.w,
            proto_msg.tool_pose_task().task_tform_desired_tool().rotation().w());
  ASSERT_EQ(ros_msg.tool_pose_task.task_tform_desired_tool.orientation.x,
            proto_msg.tool_pose_task().task_tform_desired_tool().rotation().x());
  ASSERT_EQ(ros_msg.tool_pose_task.task_tform_desired_tool.orientation.y,
            proto_msg.tool_pose_task().task_tform_desired_tool().rotation().y());
  ASSERT_EQ(ros_msg.tool_pose_task.task_tform_desired_tool.orientation.z,
            proto_msg.tool_pose_task().task_tform_desired_tool().rotation().z());
}

TEST(TestKinematicConversions, convert_bosdyn_msgs_inverse_kinematics_request_one_of_task_specification_to_proto_gaze) {
  bosdyn_msgs::msg::InverseKinematicsRequestOneOfTaskSpecification ros_msg;
  bosdyn::api::spot::InverseKinematicsRequest proto_msg;

  ros_msg.task_specification_choice =
      bosdyn_msgs::msg::InverseKinematicsRequestOneOfTaskSpecification::TASK_SPECIFICATION_TOOL_GAZE_TASK_SET;
  ros_msg.tool_gaze_task.task_tform_desired_tool_is_set = true;
  ros_msg.tool_gaze_task.task_tform_desired_tool.position.x = 0.1;
  ros_msg.tool_gaze_task.task_tform_desired_tool.position.y = 0.2;
  ros_msg.tool_gaze_task.task_tform_desired_tool.position.z = 0.3;
  ros_msg.tool_gaze_task.task_tform_desired_tool.orientation.w = 0.7987;
  ros_msg.tool_gaze_task.task_tform_desired_tool.orientation.x = 0.1912;
  ros_msg.tool_gaze_task.task_tform_desired_tool.orientation.y = 0.4336;
  ros_msg.tool_gaze_task.task_tform_desired_tool.orientation.z = 0.3709;

  kinematic_conversions::convert_bosdyn_msgs_inverse_kinematics_request_one_of_task_specification_to_proto(ros_msg,
                                                                                                           proto_msg);
  ASSERT_EQ(ros_msg.tool_gaze_task.task_tform_desired_tool.position.x,
            proto_msg.tool_gaze_task().task_tform_desired_tool().position().x());
  ASSERT_EQ(ros_msg.tool_gaze_task.task_tform_desired_tool.position.y,
            proto_msg.tool_gaze_task().task_tform_desired_tool().position().y());
  ASSERT_EQ(ros_msg.tool_gaze_task.task_tform_desired_tool.position.z,
            proto_msg.tool_gaze_task().task_tform_desired_tool().position().z());
  ASSERT_EQ(ros_msg.tool_gaze_task.task_tform_desired_tool.orientation.w,
            proto_msg.tool_gaze_task().task_tform_desired_tool().rotation().w());
  ASSERT_EQ(ros_msg.tool_gaze_task.task_tform_desired_tool.orientation.x,
            proto_msg.tool_gaze_task().task_tform_desired_tool().rotation().x());
  ASSERT_EQ(ros_msg.tool_gaze_task.task_tform_desired_tool.orientation.y,
            proto_msg.tool_gaze_task().task_tform_desired_tool().rotation().y());
  ASSERT_EQ(ros_msg.tool_gaze_task.task_tform_desired_tool.orientation.z,
            proto_msg.tool_gaze_task().task_tform_desired_tool().rotation().z());
}

}  // namespace spot_ros2