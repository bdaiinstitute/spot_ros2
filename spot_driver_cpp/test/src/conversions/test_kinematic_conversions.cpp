// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gtest/gtest.h>
#include <spot_driver_cpp/conversions/kinematic_conversions.hpp>

#include <rclcpp/rclcpp.hpp>

namespace spot_ros2::test {

///////////////////////////////////////////////////////////////////////////////
// ROS to Protobuf.

TEST(TestKinematicConversions, convertBosdynMsgsInverseKinematicsRequestToProto) {
  bosdyn_msgs::msg::InverseKinematicsRequest rosMsg;
  bosdyn::api::spot::InverseKinematicsRequest protoMsg;

  rosMsg.header_is_set = true;
  rosMsg.root_frame_name = "root_frame_name";
  rosMsg.root_tform_scene_is_set = true;
  rosMsg.scene_tform_task_is_set = true;
  rosMsg.nominal_arm_configuration.value =
      bosdyn_msgs::msg::InverseKinematicsRequestNamedArmConfiguration::ARM_CONFIG_READY;
  rosMsg.nominal_arm_configuration_overrides_is_set = true;
  rosMsg.scene_tform_body_nominal_is_set = true;

  kinematic_conversions::convertToProto(rosMsg, protoMsg);

  ASSERT_EQ(rosMsg.header_is_set, protoMsg.has_header());
  ASSERT_EQ(rosMsg.root_frame_name, protoMsg.root_frame_name());
  ASSERT_EQ(rosMsg.root_tform_scene_is_set, protoMsg.has_root_tform_scene());
  ASSERT_EQ(rosMsg.scene_tform_task_is_set, protoMsg.has_scene_tform_task());
  ASSERT_EQ(rosMsg.nominal_arm_configuration.value, protoMsg.nominal_arm_configuration());
  ASSERT_EQ(rosMsg.nominal_arm_configuration_overrides_is_set, protoMsg.has_nominal_arm_configuration_overrides());
  ASSERT_EQ(rosMsg.scene_tform_body_nominal_is_set, protoMsg.has_scene_tform_body_nominal());
}

TEST(TestKinematicConversions, convertBosdynMsgsInverseKinematicsRequestFixedStanceToProto) {
  bosdyn_msgs::msg::InverseKinematicsRequest rosMsg;
  bosdyn::api::spot::InverseKinematicsRequest protoMsg;

  rosMsg.stance_specification.fixed_stance.fl_rt_scene_is_set = true;
  rosMsg.stance_specification.fixed_stance.fl_rt_scene.x = 0.1;
  rosMsg.stance_specification.fixed_stance.fl_rt_scene.y = 0.2;
  rosMsg.stance_specification.fixed_stance.fl_rt_scene.z = 0.3;

  rosMsg.stance_specification.fixed_stance.fr_rt_scene_is_set = true;
  rosMsg.stance_specification.fixed_stance.fr_rt_scene.x = 0.1;
  rosMsg.stance_specification.fixed_stance.fr_rt_scene.y = 0.2;
  rosMsg.stance_specification.fixed_stance.fr_rt_scene.z = 0.3;

  rosMsg.stance_specification.fixed_stance.hl_rt_scene_is_set = true;
  rosMsg.stance_specification.fixed_stance.hl_rt_scene.x = 0.1;
  rosMsg.stance_specification.fixed_stance.hl_rt_scene.y = 0.2;
  rosMsg.stance_specification.fixed_stance.hl_rt_scene.z = 0.3;

  rosMsg.stance_specification.fixed_stance.hr_rt_scene_is_set = true;
  rosMsg.stance_specification.fixed_stance.hr_rt_scene.x = 0.1;
  rosMsg.stance_specification.fixed_stance.hr_rt_scene.y = 0.2;
  rosMsg.stance_specification.fixed_stance.hr_rt_scene.z = 0.3;

  kinematic_conversions::convertToProto(rosMsg.stance_specification.fixed_stance, *protoMsg.mutable_fixed_stance());

  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.fl_rt_scene_is_set, protoMsg.fixed_stance().has_fl_rt_scene());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.fl_rt_scene.x, protoMsg.fixed_stance().fl_rt_scene().x());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.fl_rt_scene.y, protoMsg.fixed_stance().fl_rt_scene().y());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.fl_rt_scene.z, protoMsg.fixed_stance().fl_rt_scene().z());

  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.fr_rt_scene_is_set, protoMsg.fixed_stance().has_fr_rt_scene());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.fr_rt_scene.x, protoMsg.fixed_stance().fr_rt_scene().x());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.fr_rt_scene.y, protoMsg.fixed_stance().fr_rt_scene().y());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.fr_rt_scene.z, protoMsg.fixed_stance().fr_rt_scene().z());

  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.hl_rt_scene_is_set, protoMsg.fixed_stance().has_hl_rt_scene());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.hl_rt_scene.x, protoMsg.fixed_stance().hl_rt_scene().x());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.hl_rt_scene.y, protoMsg.fixed_stance().hl_rt_scene().y());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.hl_rt_scene.z, protoMsg.fixed_stance().hl_rt_scene().z());

  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.hr_rt_scene_is_set, protoMsg.fixed_stance().has_hr_rt_scene());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.hr_rt_scene.x, protoMsg.fixed_stance().hr_rt_scene().x());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.hr_rt_scene.y, protoMsg.fixed_stance().hr_rt_scene().y());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.hr_rt_scene.z, protoMsg.fixed_stance().hr_rt_scene().z());
}

TEST(TestKinematicConversions, convertBosdynMsgsInverseKinematicsRequestOnGroundPlaneStanceToProto) {
  bosdyn_msgs::msg::InverseKinematicsRequest rosMsg;
  bosdyn::api::spot::InverseKinematicsRequest protoMsg;

  rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground_is_set = true;
  rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.position.x = 0.1;
  rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.position.y = 0.2;
  rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.position.z = 0.3;
  rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.w = 0.7987;
  rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.x = 0.1912;
  rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.y = 0.4336;
  rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.z = 0.3709;

  kinematic_conversions::convertToProto(rosMsg.stance_specification.on_ground_plane_stance,
                                        *protoMsg.mutable_on_ground_plane_stance());

  ASSERT_EQ(rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground_is_set,
            protoMsg.on_ground_plane_stance().has_scene_tform_ground());
  ASSERT_EQ(rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.position.x,
            protoMsg.on_ground_plane_stance().scene_tform_ground().position().x());
  ASSERT_EQ(rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.position.y,
            protoMsg.on_ground_plane_stance().scene_tform_ground().position().y());
  ASSERT_EQ(rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.position.z,
            protoMsg.on_ground_plane_stance().scene_tform_ground().position().z());
  ASSERT_EQ(rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.w,
            protoMsg.on_ground_plane_stance().scene_tform_ground().rotation().w());
  ASSERT_EQ(rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.x,
            protoMsg.on_ground_plane_stance().scene_tform_ground().rotation().x());
  ASSERT_EQ(rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.y,
            protoMsg.on_ground_plane_stance().scene_tform_ground().rotation().y());
  ASSERT_EQ(rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.z,
            protoMsg.on_ground_plane_stance().scene_tform_ground().rotation().z());
}

TEST(TestKinematicConversions, convertBosdynMsgsInverseKinematicsRequestOneOfToolSpecificationToProtoWristMounted) {
  bosdyn_msgs::msg::InverseKinematicsRequestOneOfToolSpecification rosMsg;
  bosdyn::api::spot::InverseKinematicsRequest protoMsg;

  rosMsg.wrist_mounted_tool.wrist_tform_tool_is_set = true;
  rosMsg.tool_specification_choice =
      bosdyn_msgs::msg::InverseKinematicsRequestOneOfToolSpecification::TOOL_SPECIFICATION_WRIST_MOUNTED_TOOL_SET;
  rosMsg.wrist_mounted_tool.wrist_tform_tool.position.x = 0.1;
  rosMsg.wrist_mounted_tool.wrist_tform_tool.position.y = 0.2;
  rosMsg.wrist_mounted_tool.wrist_tform_tool.position.z = 0.3;
  rosMsg.wrist_mounted_tool.wrist_tform_tool.orientation.w = 0.7987;
  rosMsg.wrist_mounted_tool.wrist_tform_tool.orientation.x = 0.1912;
  rosMsg.wrist_mounted_tool.wrist_tform_tool.orientation.y = 0.4336;
  rosMsg.wrist_mounted_tool.wrist_tform_tool.orientation.z = 0.3709;

  kinematic_conversions::convertToProto(rosMsg, protoMsg);
  ASSERT_EQ(rosMsg.wrist_mounted_tool.wrist_tform_tool.position.x,
            protoMsg.wrist_mounted_tool().wrist_tform_tool().position().x());
  ASSERT_EQ(rosMsg.wrist_mounted_tool.wrist_tform_tool.position.y,
            protoMsg.wrist_mounted_tool().wrist_tform_tool().position().y());
  ASSERT_EQ(rosMsg.wrist_mounted_tool.wrist_tform_tool.position.z,
            protoMsg.wrist_mounted_tool().wrist_tform_tool().position().z());
  ASSERT_EQ(rosMsg.wrist_mounted_tool.wrist_tform_tool.orientation.w,
            protoMsg.wrist_mounted_tool().wrist_tform_tool().rotation().w());
  ASSERT_EQ(rosMsg.wrist_mounted_tool.wrist_tform_tool.orientation.x,
            protoMsg.wrist_mounted_tool().wrist_tform_tool().rotation().x());
  ASSERT_EQ(rosMsg.wrist_mounted_tool.wrist_tform_tool.orientation.y,
            protoMsg.wrist_mounted_tool().wrist_tform_tool().rotation().y());
  ASSERT_EQ(rosMsg.wrist_mounted_tool.wrist_tform_tool.orientation.z,
            protoMsg.wrist_mounted_tool().wrist_tform_tool().rotation().z());
}

TEST(TestKinematicConversions, convertBosdynMsgsInverseKinematicsRequestOneOfToolSpecificationToProtoBodyMounted) {
  bosdyn_msgs::msg::InverseKinematicsRequestOneOfToolSpecification rosMsg;
  bosdyn::api::spot::InverseKinematicsRequest protoMsg;

  rosMsg.body_mounted_tool.body_tform_tool_is_set = true;
  rosMsg.tool_specification_choice =
      bosdyn_msgs::msg::InverseKinematicsRequestOneOfToolSpecification::TOOL_SPECIFICATION_BODY_MOUNTED_TOOL_SET;
  rosMsg.body_mounted_tool.body_tform_tool.position.x = 0.1;
  rosMsg.body_mounted_tool.body_tform_tool.position.y = 0.2;
  rosMsg.body_mounted_tool.body_tform_tool.position.z = 0.3;
  rosMsg.body_mounted_tool.body_tform_tool.orientation.w = 0.7987;
  rosMsg.body_mounted_tool.body_tform_tool.orientation.x = 0.1912;
  rosMsg.body_mounted_tool.body_tform_tool.orientation.y = 0.4336;
  rosMsg.body_mounted_tool.body_tform_tool.orientation.z = 0.3709;

  kinematic_conversions::convertToProto(rosMsg, protoMsg);
  ASSERT_EQ(rosMsg.body_mounted_tool.body_tform_tool.position.x,
            protoMsg.body_mounted_tool().body_tform_tool().position().x());
  ASSERT_EQ(rosMsg.body_mounted_tool.body_tform_tool.position.y,
            protoMsg.body_mounted_tool().body_tform_tool().position().y());
  ASSERT_EQ(rosMsg.body_mounted_tool.body_tform_tool.position.z,
            protoMsg.body_mounted_tool().body_tform_tool().position().z());
  ASSERT_EQ(rosMsg.body_mounted_tool.body_tform_tool.orientation.w,
            protoMsg.body_mounted_tool().body_tform_tool().rotation().w());
  ASSERT_EQ(rosMsg.body_mounted_tool.body_tform_tool.orientation.x,
            protoMsg.body_mounted_tool().body_tform_tool().rotation().x());
  ASSERT_EQ(rosMsg.body_mounted_tool.body_tform_tool.orientation.y,
            protoMsg.body_mounted_tool().body_tform_tool().rotation().y());
  ASSERT_EQ(rosMsg.body_mounted_tool.body_tform_tool.orientation.z,
            protoMsg.body_mounted_tool().body_tform_tool().rotation().z());
}

TEST(TestKinematicConversions, convertBosdynMsgsInverseKinematicsRequestOneOfTaskSpecificationToProto) {
  bosdyn_msgs::msg::InverseKinematicsRequestOneOfTaskSpecification rosMsg;
  bosdyn::api::spot::InverseKinematicsRequest protoMsg;

  rosMsg.task_specification_choice =
      bosdyn_msgs::msg::InverseKinematicsRequestOneOfTaskSpecification::TASK_SPECIFICATION_TOOL_POSE_TASK_SET;
  rosMsg.tool_pose_task.task_tform_desired_tool_is_set = true;
  rosMsg.tool_pose_task.task_tform_desired_tool.position.x = 0.1;
  rosMsg.tool_pose_task.task_tform_desired_tool.position.y = 0.2;
  rosMsg.tool_pose_task.task_tform_desired_tool.position.z = 0.3;
  rosMsg.tool_pose_task.task_tform_desired_tool.orientation.w = 0.7987;
  rosMsg.tool_pose_task.task_tform_desired_tool.orientation.x = 0.1912;
  rosMsg.tool_pose_task.task_tform_desired_tool.orientation.y = 0.4336;
  rosMsg.tool_pose_task.task_tform_desired_tool.orientation.z = 0.3709;

  kinematic_conversions::convertToProto(rosMsg, protoMsg);
  ASSERT_EQ(rosMsg.tool_pose_task.task_tform_desired_tool.position.x,
            protoMsg.tool_pose_task().task_tform_desired_tool().position().x());
  ASSERT_EQ(rosMsg.tool_pose_task.task_tform_desired_tool.position.y,
            protoMsg.tool_pose_task().task_tform_desired_tool().position().y());
  ASSERT_EQ(rosMsg.tool_pose_task.task_tform_desired_tool.position.z,
            protoMsg.tool_pose_task().task_tform_desired_tool().position().z());
  ASSERT_EQ(rosMsg.tool_pose_task.task_tform_desired_tool.orientation.w,
            protoMsg.tool_pose_task().task_tform_desired_tool().rotation().w());
  ASSERT_EQ(rosMsg.tool_pose_task.task_tform_desired_tool.orientation.x,
            protoMsg.tool_pose_task().task_tform_desired_tool().rotation().x());
  ASSERT_EQ(rosMsg.tool_pose_task.task_tform_desired_tool.orientation.y,
            protoMsg.tool_pose_task().task_tform_desired_tool().rotation().y());
  ASSERT_EQ(rosMsg.tool_pose_task.task_tform_desired_tool.orientation.z,
            protoMsg.tool_pose_task().task_tform_desired_tool().rotation().z());
}

TEST(TestKinematicConversions, convertBosdynMsgsInverseKinematicsRequestOneOfTaskSpecificationToProtoGaze) {
  bosdyn_msgs::msg::InverseKinematicsRequestOneOfTaskSpecification rosMsg;
  bosdyn::api::spot::InverseKinematicsRequest protoMsg;

  rosMsg.task_specification_choice =
      bosdyn_msgs::msg::InverseKinematicsRequestOneOfTaskSpecification::TASK_SPECIFICATION_TOOL_GAZE_TASK_SET;
  rosMsg.tool_gaze_task.task_tform_desired_tool_is_set = true;
  rosMsg.tool_gaze_task.task_tform_desired_tool.position.x = 0.1;
  rosMsg.tool_gaze_task.task_tform_desired_tool.position.y = 0.2;
  rosMsg.tool_gaze_task.task_tform_desired_tool.position.z = 0.3;
  rosMsg.tool_gaze_task.task_tform_desired_tool.orientation.w = 0.7987;
  rosMsg.tool_gaze_task.task_tform_desired_tool.orientation.x = 0.1912;
  rosMsg.tool_gaze_task.task_tform_desired_tool.orientation.y = 0.4336;
  rosMsg.tool_gaze_task.task_tform_desired_tool.orientation.z = 0.3709;

  kinematic_conversions::convertToProto(rosMsg, protoMsg);
  ASSERT_EQ(rosMsg.tool_gaze_task.task_tform_desired_tool.position.x,
            protoMsg.tool_gaze_task().task_tform_desired_tool().position().x());
  ASSERT_EQ(rosMsg.tool_gaze_task.task_tform_desired_tool.position.y,
            protoMsg.tool_gaze_task().task_tform_desired_tool().position().y());
  ASSERT_EQ(rosMsg.tool_gaze_task.task_tform_desired_tool.position.z,
            protoMsg.tool_gaze_task().task_tform_desired_tool().position().z());
  ASSERT_EQ(rosMsg.tool_gaze_task.task_tform_desired_tool.orientation.w,
            protoMsg.tool_gaze_task().task_tform_desired_tool().rotation().w());
  ASSERT_EQ(rosMsg.tool_gaze_task.task_tform_desired_tool.orientation.x,
            protoMsg.tool_gaze_task().task_tform_desired_tool().rotation().x());
  ASSERT_EQ(rosMsg.tool_gaze_task.task_tform_desired_tool.orientation.y,
            protoMsg.tool_gaze_task().task_tform_desired_tool().rotation().y());
  ASSERT_EQ(rosMsg.tool_gaze_task.task_tform_desired_tool.orientation.z,
            protoMsg.tool_gaze_task().task_tform_desired_tool().rotation().z());
}

}  // namespace spot_ros2::test
