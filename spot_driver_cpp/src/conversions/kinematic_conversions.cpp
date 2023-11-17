// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/conversions/kinematic_conversions.hpp>

#include <spot_driver_cpp/conversions/common_conversions.hpp>

namespace spot_ros2::kinematic_conversions {

void convert_bosdyn_msgs_inverse_kinematics_request_to_proto(const bosdyn_msgs::msg::InverseKinematicsRequest& ros_msg,
                                                             bosdyn::api::spot::InverseKinematicsRequest& proto) {
  if (ros_msg.header_is_set) {
    common_conversions::convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, *proto.mutable_header());
  }
  proto.set_root_frame_name(ros_msg.root_frame_name);
  if (ros_msg.root_tform_scene_is_set) {
    common_conversions::convert_geometry_msgs_pose_to_proto(ros_msg.root_tform_scene,
                                                            *proto.mutable_root_tform_scene());
  }
  if (ros_msg.scene_tform_task_is_set) {
    common_conversions::convert_geometry_msgs_pose_to_proto(ros_msg.scene_tform_task,
                                                            *proto.mutable_scene_tform_task());
  }

  // proto.nominal_arm_configuration = ros_msg.nominal_arm_configuration.value

  if (ros_msg.nominal_arm_configuration_overrides_is_set) {
    common_conversions::convert_bosdyn_msgs_arm_joint_position_to_proto(
        ros_msg.nominal_arm_configuration_overrides, *proto.mutable_nominal_arm_configuration_overrides());
  }
  if (ros_msg.scene_tform_body_nominal_is_set) {
    common_conversions::convert_geometry_msgs_pose_to_proto(ros_msg.scene_tform_body_nominal,
                                                            *proto.mutable_scene_tform_body_nominal());
  }

  // convert_bosdyn_msgs_inverse_kinematics_request_one_of_stance_specification_to_proto(ros_msg.stance_specification,
  // proto)
  // convert_bosdyn_msgs_inverse_kinematics_request_one_of_tool_specification_to_proto(ros_msg.tool_specification,
  // proto)
  // convert_bosdyn_msgs_inverse_kinematics_request_one_of_task_specification_to_proto(ros_msg.task_specification,
  // proto)
}

void convert_proto_to_bosdyn_msgs_inverse_kinematics_response(const bosdyn::api::spot::InverseKinematicsResponse& proto,
                                                              bosdyn_msgs::msg::InverseKinematicsResponse& ros_msg) {}

}  // namespace spot_ros2::kinematic_conversions
