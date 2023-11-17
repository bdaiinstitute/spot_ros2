// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/kinematic_utils.hpp>

namespace spot_ros2::kinematic_utils {

void convert_time_to_proto(const builtin_interfaces::msg::Time& ros_msg, google::protobuf::Timestamp& proto) {
  proto.set_seconds(ros_msg.sec);
  proto.set_nanos(ros_msg.nanosec);
}

void convert_request_header_to_proto(const bosdyn_msgs::msg::RequestHeader& ros_msg,
                                     bosdyn::api::RequestHeader& proto) {
  if (ros_msg.request_timestamp_is_set) {
    convert_time_to_proto(ros_msg.request_timestamp, *proto.mutable_request_timestamp());
  }
  proto.set_client_name(ros_msg.client_name);
  proto.set_disable_rpc_logging(true);
}

void convert_inverse_kinematics_request_to_proto(const bosdyn_msgs::msg::InverseKinematicsRequest& ros_msg,
                                                 bosdyn::api::spot::InverseKinematicsRequest& proto) {
  if (ros_msg.header_is_set) {
    convert_request_header_to_proto(ros_msg.header, *proto.mutable_header());
  }
  // proto.root_frame_name = ros_msg.root_frame_name if ros_msg
  //                                .root_tform_scene_is_set
  //        : convert_geometry_msgs_pose_to_proto(ros_msg.root_tform_scene, proto.root_tform_scene) if ros_msg
  //                                .scene_tform_task_is_set
  //        : convert_geometry_msgs_pose_to_proto(ros_msg.scene_tform_task, proto.scene_tform_task)
  //              proto.nominal_arm_configuration =
  //        ros_msg.nominal_arm_configuration.value if ros_msg
  //            .nominal_arm_configuration_overrides_is_set
  //        : convert_bosdyn_msgs_arm_joint_position_to_proto(
  //                ros_msg.nominal_arm_configuration_overrides,
  //                proto.nominal_arm_configuration_overrides) if ros_msg.scene_tform_body_nominal_is_set
  //        : convert_geometry_msgs_pose_to_proto(ros_msg.scene_tform_body_nominal, proto.scene_tform_body_nominal)
  //              convert_bosdyn_msgs_inverse_kinematics_request_one_of_stance_specification_to_proto(
  //                  ros_msg.stance_specification, proto)
  //                  convert_bosdyn_msgs_inverse_kinematics_request_one_of_tool_specification_to_proto(
  //                      ros_msg.tool_specification, proto)
  //                      convert_bosdyn_msgs_inverse_kinematics_request_one_of_task_specification_to_proto(
  //                          ros_msg.task_specification, proto)
}

void convert_proto_to_inverse_kinematics_response(const bosdyn::api::spot::InverseKinematicsResponse& proto,
                                                  bosdyn_msgs::msg::InverseKinematicsResponse& ros_msg) {}

}  // namespace spot_ros2::kinematic_utils
