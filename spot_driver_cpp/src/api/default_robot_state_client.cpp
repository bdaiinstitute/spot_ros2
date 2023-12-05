// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/api/default_robot_state_client.hpp>
#include <bosdyn/api/robot_state.pb.h>
#include <bosdyn/math/frame_helpers.h>
#include <bosdyn/api/geometry.pb.h>
#include <string_view>

namespace {
static const std::map<std::string, std::string> kFriendlyJointNames = {
  {"fl.hx", "front_left_hip_x"},
  {"fl.hy", "front_left_hip_y"},
  {"fl.kn", "front_left_knee"},
  {"fr.hx", "front_right_hip_x"},
  {"fr.hy", "front_right_hip_y"},
  {"fr.kn", "front_right_knee"},
  {"hl.hx", "rear_left_hip_x"},
  {"hl.hy", "rear_left_hip_y"},
  {"hl.kn", "rear_left_knee"},
  {"hr.hx", "rear_right_hip_x"},
  {"hr.hy", "rear_right_hip_y"},
  {"hr.kn", "rear_right_knee"},
  {"arm0.sh0", "arm_sh0"},
  {"arm0.sh1", "arm_sh1"},
  {"arm0.hr0", "arm_hr0"},
  {"arm0.el0", "arm_el0"},
  {"arm0.el1", "arm_el1"},
  {"arm0.wr0", "arm_wr0"},
  {"arm0.wr1", "arm_wr1"},
  {"arm0.f1x", "arm_f1x"},
};
                                                      
spot_msgs::msg::BatteryStateArray GetBatteryState(const ::bosdyn::api::RobotState &robot_state){}
spot_msgs::msg::WiFiState GetWifiState(const ::bosdyn::api::RobotState &robot_state){}
spot_msgs::msg::FootState GetFootState(const ::bosdyn::api::RobotState &robot_state){}
spot_msgs::msg::EStopStateArray GetEstopStates(const ::bosdyn::api::RobotState &robot_state){}

std::optional<sensor_msgs::msg::JointState> GetJointStates(const ::bosdyn::api::RobotState &robot_state, const builtin_interfaces::msg::Time &local_time, const std::string &prefix){
  if(robot_state.kinematic_state_is_set()){
    sensor_msgs::msg::JointState joint_states;
    joint_states.header.stamp = local_time;

    for(auto joint : robot_state.kinematic_state().joint_states()){
        joint_states.name.append(std::string(prefix + kFriendlyJointNames[joint.name()]));
        joint_states.position.append(joint.position());
        joint_states.velocity.append(joint.velocity());
        joint_states.effort.append(joint.effort());
    }

    return joint_states;
  }
  return {};
}

std::optional<tf2_msgs::msg::TFMessage> GetTf(const ::bosdyn::api::RobotState &robot_state, const builtin_interfaces::msg::Time &local_time, const std::string &prefix, const std::string &inverse_target_frame_id){
  if(robot_state.kinematic_state_is_set()){
    tf2_msgs::msg::TFMessage tf_msg;
    
    for (const auto& [child_frame_id, transform] :
      robot_state.kinematic_state().transforms_snapshot().child_to_parent_edge_map()) {
      if(inverse_target_frame_id == std::string(prefix + child_frame_id)){
        const auto inversed_tf = ~::bosdyn::api::SE3Pose(transform.parent_tform_child());
        tf_msg.transforms.append(toTransformStamped(
          inversed_tf,
          std::string(prefix + child_frame_id),
          std::string(prefix + transform.parent_frame_name()), 
          local_time));
      }
      else {
        tf_msg.transforms.append(toTransformStamped(transform.parent_tform_child(),
          std::string(prefix + transform.parent_frame_name()), 
          std::string(prefix + child_frame_id), 
          local_time));
      }
    }
    return tf_msg;
  }

  return {};
}

std::optional<geometry_msgs::msg::TwistWithCovariance> GetOdomTwist(const ::bosdyn::api::RobotState &robot_state, const builtin_interfaces::msg::Time &local_time, const std::string &prefix){
  if(robot_state.kinematic_state_is_set()){
    geometry_msgs::msg::TwistWithCovariance odom_twist_msg;

    odom_twist_msg.header.stamp = local_time;
    odom_twist_msg.twist.twist.linear.x = robot_state.kinematic_state().velocity_of_body_in_odom().linear().x();
    odom_twist_msg.twist.twist.linear.y = robot_state.kinematic_state().velocity_of_body_in_odom().linear().y();
    odom_twist_msg.twist.twist.linear.z = robot_state.kinematic_state().velocity_of_body_in_odom().linear().z();
    odom_twist_msg.twist.twist.angular.x = robot_state.kinematic_state().velocity_of_body_in_odom().linear().x();
    odom_twist_msg.twist.twist.angular.y = robot_state.kinematic_state().velocity_of_body_in_odom().angular().y();
    odom_twist_msg.twist.twist.angular.z = robot_state.kinematic_state().velocity_of_body_in_odom().angular().z();

    return odom_twist_msg;
  }
  return {};
}

std::optional<nav_msgs::msg::Odometry> GetOdom(const ::bosdyn::api::RobotState &robot_state, const builtin_interfaces::msg::Time &local_time, const std::string &prefix, bool is_using_vision){
  if(robot_state.kinematic_state_is_set()){
    nav_msgs::msg::Odometry odom_msg;
    ::bosdyn::api::SE3Pose tf_body_pose;
    geometry_msgs::msg::PoseWithCovariance pose_odom_msg;


    odom_msg.header.stamp = local_time;
    if(is_using_vision){
      odom_msg.header.frame_id = prefix + "vision";
      ::bosdyn::api::GetWorldTformBody(robot_state.kinematic_state().transforms_snapshot(), &tf_body_pose);
    }
    else{
      odom_msg.header.frame_id = prefix + "odom";
      ::bosdyn::api::GetOdomTformBody(robot_state.kinematic_state().transforms_snapshot(), &tf_body_pose);
    }
    odom_msg.child_frame_id = prefix + "body";

    pose_odom_msg.pose.position.x = tf_body_pose.position.x;
    pose_odom_msg.pose.position.y = tf_body_pose.position.y;
    pose_odom_msg.pose.position.z = tf_body_pose.position.z;
    pose_odom_msg.pose.orientation.x = tf_body_pose.orientation.x;
    pose_odom_msg.pose.orientation.y = tf_body_pose.orientation.y;
    pose_odom_msg.pose.orientation.z = tf_body_pose.orientation.z;
    pose_odom_msg.pose.orientation.w = tf_body_pose.orientation.w;

    odom_msg.pose = pose_odom_msg;
    odom_msg.twist = GetOdomTwist(robot_state, local_time, prefix).value().value;
    return odom_msg;

  }
  return {};
}

std::optional<spot_msgs::msg::PowerState> GetPowerState(const ::bosdyn::api::RobotState &robot_state){}
std::optional<spot_msgs::msg::SystemFaultState> GetSystemFaultState(const ::bosdyn::api::RobotState &robot_state){}
std::optional<bosdyn_msgs::msg::ManipulationFeedbackState> GetManipulationState(const ::bosdyn::api::RobotState &robot_state){}
std::optional<geometry_msgs::msg::Vector3Stamped> GetEndEffectorForce(const ::bosdyn::api::RobotState &robot_state){}
std::optional<spot_msgs::msg::BehaviorFaultState> GetBehaviorFaultState(const ::bosdyn::api::RobotState &robot_state){}
}

namespace spot_ros2 {

DefaultRobotStateClient::DefaultRobotStateClient(::bosdyn::client::RobotStateClient* client, 
                          std::shared_ptr<TimeSyncApi> time_sync_api,
                          const std::string &robot_name)
                          : client_{client}
                          , time_sync_api_{time_sync_api}
                          , frame_prefix_{robot_name.empty() ? "" : robot_name + "/"}

tl::expected<RobotState, std::string> DefaultRobotStateClient::getRobotState(::bosdyn::api::GetRobotStateRequest request) {
  std::shared_future<::bosdyn::client::RobotStateResultType> get_robot_state_result_future =
      client_->GetRobotState(request);

  ::bosdyn::client::RobotStateResultType get_robot_state_result = get_robot_state_result_future.get();
  if (!get_robot_state_result.status || !get_robot_state_result.response.has_robot_state()) {
    return tl::make_unexpected("Failed to get robot state: " + get_robot_state_result.status.DebugString());
  }

  const auto clock_skew_result = time_sync_api_->getClockSkew();
  if (!clock_skew_result) {
    return tl::make_unexpected("Failed to get latest clock skew: " + clock_skew_result.error());
  }

  const auto robot_state_proto = get_robot_state_result.response.robot_state();

  RobotState out;
  return out;
}

}
