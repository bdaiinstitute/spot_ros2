// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gtest/gtest.h>
#include <spot_driver/conversions/kinematic_conversions.hpp>

#include <rclcpp/rclcpp.hpp>

namespace spot_ros2::test {

///////////////////////////////////////////////////////////////////////////////
// ROS to Protobuf.

TEST(TestKinematicConversions, convertBosdynMsgsInverseKinematicsRequestToProto) {
  bosdyn_spot_api_msgs::msg::InverseKinematicsRequest rosMsg;
  bosdyn::api::spot::InverseKinematicsRequest protoMsg;

  // All fields assumed set by default.
  rosMsg.root_frame_name = "root_frame_name";
  rosMsg.nominal_arm_configuration.value =
      bosdyn_spot_api_msgs::msg::InverseKinematicsRequestNamedArmConfiguration::ARM_CONFIG_READY;

  convertToProto(rosMsg, protoMsg);

  ASSERT_TRUE(protoMsg.has_header());
  ASSERT_EQ(rosMsg.root_frame_name, protoMsg.root_frame_name());
  ASSERT_TRUE(protoMsg.has_root_tform_scene());
  ASSERT_TRUE(protoMsg.has_scene_tform_task());
  ASSERT_EQ(rosMsg.nominal_arm_configuration.value, protoMsg.nominal_arm_configuration());
  ASSERT_TRUE(protoMsg.has_nominal_arm_configuration_overrides());
  ASSERT_TRUE(protoMsg.has_scene_tform_body_nominal());
}

TEST(TestKinematicConversions, convertBosdynMsgsInverseKinematicsRequestFixedStanceToProto) {
  bosdyn_spot_api_msgs::msg::InverseKinematicsRequest rosMsg;
  bosdyn::api::spot::InverseKinematicsRequest protoMsg;

  // All fields assumed set by default.
  rosMsg.stance_specification.fixed_stance.fl_rt_scene.x = 0.1;
  rosMsg.stance_specification.fixed_stance.fl_rt_scene.y = 0.2;
  rosMsg.stance_specification.fixed_stance.fl_rt_scene.z = 0.3;

  rosMsg.stance_specification.fixed_stance.fr_rt_scene.x = 0.1;
  rosMsg.stance_specification.fixed_stance.fr_rt_scene.y = 0.2;
  rosMsg.stance_specification.fixed_stance.fr_rt_scene.z = 0.3;

  rosMsg.stance_specification.fixed_stance.hl_rt_scene.x = 0.1;
  rosMsg.stance_specification.fixed_stance.hl_rt_scene.y = 0.2;
  rosMsg.stance_specification.fixed_stance.hl_rt_scene.z = 0.3;

  rosMsg.stance_specification.fixed_stance.hr_rt_scene.x = 0.1;
  rosMsg.stance_specification.fixed_stance.hr_rt_scene.y = 0.2;
  rosMsg.stance_specification.fixed_stance.hr_rt_scene.z = 0.3;

  convertToProto(rosMsg.stance_specification.fixed_stance, *protoMsg.mutable_fixed_stance());

  ASSERT_TRUE(protoMsg.fixed_stance().has_fl_rt_scene());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.fl_rt_scene.x, protoMsg.fixed_stance().fl_rt_scene().x());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.fl_rt_scene.y, protoMsg.fixed_stance().fl_rt_scene().y());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.fl_rt_scene.z, protoMsg.fixed_stance().fl_rt_scene().z());

  ASSERT_TRUE(protoMsg.fixed_stance().has_fr_rt_scene());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.fr_rt_scene.x, protoMsg.fixed_stance().fr_rt_scene().x());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.fr_rt_scene.y, protoMsg.fixed_stance().fr_rt_scene().y());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.fr_rt_scene.z, protoMsg.fixed_stance().fr_rt_scene().z());

  ASSERT_TRUE(protoMsg.fixed_stance().has_hl_rt_scene());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.hl_rt_scene.x, protoMsg.fixed_stance().hl_rt_scene().x());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.hl_rt_scene.y, protoMsg.fixed_stance().hl_rt_scene().y());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.hl_rt_scene.z, protoMsg.fixed_stance().hl_rt_scene().z());

  ASSERT_TRUE(protoMsg.fixed_stance().has_hr_rt_scene());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.hr_rt_scene.x, protoMsg.fixed_stance().hr_rt_scene().x());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.hr_rt_scene.y, protoMsg.fixed_stance().hr_rt_scene().y());
  ASSERT_EQ(rosMsg.stance_specification.fixed_stance.hr_rt_scene.z, protoMsg.fixed_stance().hr_rt_scene().z());
}

TEST(TestKinematicConversions, convertBosdynMsgsInverseKinematicsRequestOnGroundPlaneStanceToProto) {
  bosdyn_spot_api_msgs::msg::InverseKinematicsRequest rosMsg;
  bosdyn::api::spot::InverseKinematicsRequest protoMsg;

  // Field assumed set by default.
  rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.position.x = 0.1;
  rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.position.y = 0.2;
  rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.position.z = 0.3;
  rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.w = 0.7987;
  rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.x = 0.1912;
  rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.y = 0.4336;
  rosMsg.stance_specification.on_ground_plane_stance.scene_tform_ground.orientation.z = 0.3709;

  convertToProto(rosMsg.stance_specification.on_ground_plane_stance, *protoMsg.mutable_on_ground_plane_stance());

  ASSERT_TRUE(protoMsg.on_ground_plane_stance().has_scene_tform_ground());
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
  using InverseKinematicsRequestOneOfToolSpecification =
      bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOneOfToolSpecification;
  InverseKinematicsRequestOneOfToolSpecification rosMsg;
  bosdyn::api::spot::InverseKinematicsRequest protoMsg;

  rosMsg.which = InverseKinematicsRequestOneOfToolSpecification::TOOL_SPECIFICATION_WRIST_MOUNTED_TOOL_SET;
  rosMsg.wrist_mounted_tool.wrist_tform_tool.position.x = 0.1;
  rosMsg.wrist_mounted_tool.wrist_tform_tool.position.y = 0.2;
  rosMsg.wrist_mounted_tool.wrist_tform_tool.position.z = 0.3;
  rosMsg.wrist_mounted_tool.wrist_tform_tool.orientation.w = 0.7987;
  rosMsg.wrist_mounted_tool.wrist_tform_tool.orientation.x = 0.1912;
  rosMsg.wrist_mounted_tool.wrist_tform_tool.orientation.y = 0.4336;
  rosMsg.wrist_mounted_tool.wrist_tform_tool.orientation.z = 0.3709;

  convertToProto(rosMsg, protoMsg);
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
  using InverseKinematicsRequestOneOfToolSpecification =
      bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOneOfToolSpecification;
  InverseKinematicsRequestOneOfToolSpecification rosMsg;
  bosdyn::api::spot::InverseKinematicsRequest protoMsg;

  rosMsg.which = InverseKinematicsRequestOneOfToolSpecification::TOOL_SPECIFICATION_BODY_MOUNTED_TOOL_SET;
  // Field assumed set by default.
  rosMsg.body_mounted_tool.body_tform_tool.position.x = 0.1;
  rosMsg.body_mounted_tool.body_tform_tool.position.y = 0.2;
  rosMsg.body_mounted_tool.body_tform_tool.position.z = 0.3;
  rosMsg.body_mounted_tool.body_tform_tool.orientation.w = 0.7987;
  rosMsg.body_mounted_tool.body_tform_tool.orientation.x = 0.1912;
  rosMsg.body_mounted_tool.body_tform_tool.orientation.y = 0.4336;
  rosMsg.body_mounted_tool.body_tform_tool.orientation.z = 0.3709;

  convertToProto(rosMsg, protoMsg);
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
  using InverseKinematicsRequestOneOfTaskSpecification =
      bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOneOfTaskSpecification;
  InverseKinematicsRequestOneOfTaskSpecification rosMsg;
  bosdyn::api::spot::InverseKinematicsRequest protoMsg;

  rosMsg.which = InverseKinematicsRequestOneOfTaskSpecification::TASK_SPECIFICATION_TOOL_POSE_TASK_SET;
  // Field assumed set by default.
  rosMsg.tool_pose_task.task_tform_desired_tool.position.x = 0.1;
  rosMsg.tool_pose_task.task_tform_desired_tool.position.y = 0.2;
  rosMsg.tool_pose_task.task_tform_desired_tool.position.z = 0.3;
  rosMsg.tool_pose_task.task_tform_desired_tool.orientation.w = 0.7987;
  rosMsg.tool_pose_task.task_tform_desired_tool.orientation.x = 0.1912;
  rosMsg.tool_pose_task.task_tform_desired_tool.orientation.y = 0.4336;
  rosMsg.tool_pose_task.task_tform_desired_tool.orientation.z = 0.3709;

  convertToProto(rosMsg, protoMsg);
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
  using InverseKinematicsRequestOneOfTaskSpecification =
      bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOneOfTaskSpecification;
  InverseKinematicsRequestOneOfTaskSpecification rosMsg;
  bosdyn::api::spot::InverseKinematicsRequest protoMsg;

  rosMsg.which = InverseKinematicsRequestOneOfTaskSpecification::TASK_SPECIFICATION_TOOL_GAZE_TASK_SET;
  // Field assumed set by default.
  rosMsg.tool_gaze_task.task_tform_desired_tool.position.x = 0.1;
  rosMsg.tool_gaze_task.task_tform_desired_tool.position.y = 0.2;
  rosMsg.tool_gaze_task.task_tform_desired_tool.position.z = 0.3;
  rosMsg.tool_gaze_task.task_tform_desired_tool.orientation.w = 0.7987;
  rosMsg.tool_gaze_task.task_tform_desired_tool.orientation.x = 0.1912;
  rosMsg.tool_gaze_task.task_tform_desired_tool.orientation.y = 0.4336;
  rosMsg.tool_gaze_task.task_tform_desired_tool.orientation.z = 0.3709;

  convertToProto(rosMsg, protoMsg);
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

TEST(TestKinematicConversions, convertProtoToBosdynMsgsKinematicState) {
  bosdyn::api::KinematicState protoMsg;
  bosdyn_api_msgs::msg::KinematicState rosMsg;

  auto* joint_state = protoMsg.add_joint_states();
  joint_state->set_name("shoulder");
  joint_state->mutable_position()->set_value(0.0);
  joint_state->mutable_velocity()->set_value(0.1);
  joint_state->mutable_acceleration()->set_value(0.2);
  joint_state->mutable_load()->set_value(0.5);
  protoMsg.mutable_acquisition_timestamp()->set_seconds(1);
  auto* snapshot = protoMsg.mutable_transforms_snapshot();
  auto& edge = (*snapshot->mutable_child_to_parent_edge_map())["arm"];
  edge.set_parent_frame_name("torso");
  edge.mutable_parent_tform_child()->mutable_position()->set_x(1.0);
  protoMsg.mutable_velocity_of_body_in_vision()->mutable_linear()->set_x(1.0);
  protoMsg.mutable_velocity_of_body_in_odom()->mutable_linear()->set_y(1.0);

  convertToRos(protoMsg, rosMsg);

  ASSERT_EQ(protoMsg.joint_states_size(), static_cast<int>(rosMsg.joint_states.size()));
  ASSERT_EQ(protoMsg.joint_states(0).name(), rosMsg.joint_states[0].name);
  ASSERT_EQ(protoMsg.joint_states(0).position().value(), rosMsg.joint_states[0].position.data);
  ASSERT_EQ(protoMsg.joint_states(0).velocity().value(), rosMsg.joint_states[0].velocity.data);
  ASSERT_EQ(protoMsg.joint_states(0).acceleration().value(), rosMsg.joint_states[0].acceleration.data);
  ASSERT_EQ(protoMsg.joint_states(0).load().value(), rosMsg.joint_states[0].load.data);
  ASSERT_EQ(protoMsg.transforms_snapshot().child_to_parent_edge_map().size(),
            rosMsg.transforms_snapshot.child_to_parent_edge_map.size());
  ASSERT_EQ(rosMsg.transforms_snapshot.child_to_parent_edge_map[0].key, "arm");
  ASSERT_EQ(protoMsg.transforms_snapshot().child_to_parent_edge_map().at("arm").parent_frame_name(),
            rosMsg.transforms_snapshot.child_to_parent_edge_map[0].value.parent_frame_name);
  ASSERT_EQ(protoMsg.transforms_snapshot().child_to_parent_edge_map().at("arm").parent_tform_child().position().x(),
            rosMsg.transforms_snapshot.child_to_parent_edge_map[0].value.parent_tform_child.position.x);
  ASSERT_EQ(protoMsg.acquisition_timestamp().seconds(), rosMsg.acquisition_timestamp.sec);
  ASSERT_EQ(protoMsg.velocity_of_body_in_vision().linear().x(), rosMsg.velocity_of_body_in_vision.linear.x);
  ASSERT_EQ(protoMsg.velocity_of_body_in_odom().linear().y(), rosMsg.velocity_of_body_in_odom.linear.y);
}

}  // namespace spot_ros2::test
