// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <bosdyn/math/frame_helpers.h>
#include <bosdyn/math/proto_math.h>
#include <builtin_interfaces/msg/duration.hpp>
#include <optional>
#include <spot_driver_cpp/api/time_sync_api.hpp>
#include <spot_driver_cpp/conversions/geometry.hpp>
#include <spot_driver_cpp/conversions/robot_state.hpp>

namespace spot_ros2 {

spot_msgs::msg::BatteryStateArray getBatteryStates(const ::bosdyn::api::RobotState& robot_state,
                                                   const google::protobuf::Duration& clock_skew) {
  spot_msgs::msg::BatteryStateArray battery_states;

  for (const auto& battery : robot_state.battery_states()) {
    spot_msgs::msg::BatteryState battery_state;

    battery_state.header.stamp = applyClockSkew(battery.timestamp(), clock_skew);
    battery_state.identifier = battery.identifier();
    battery_state.charge_percentage = battery.charge_percentage().value();
    battery_state.estimated_runtime = builtin_interfaces::build<builtin_interfaces::msg::Duration>()
                                          .sec(battery.estimated_runtime().seconds())
                                          .nanosec(battery.estimated_runtime().nanos());
    battery_state.current = battery.current().value();
    battery_state.voltage = battery.voltage().value();
    for (const auto& temp : battery.temperatures()) {
      battery_state.temperatures.push_back(temp);
    }
    battery_state.status = battery.status();
    battery_states.battery_states.push_back(battery_state);
  }

  return battery_states;
}

spot_msgs::msg::WiFiState getWifiState(const ::bosdyn::api::RobotState& robot_state) {
  spot_msgs::msg::WiFiState wifi_state;

  for (const auto& comm_state : robot_state.comms_states()) {
    if (comm_state.has_wifi_state()) {
      wifi_state.current_mode = comm_state.wifi_state().current_mode();
      wifi_state.essid = comm_state.wifi_state().essid();
    }
  }

  return wifi_state;
}

spot_msgs::msg::FootStateArray getFootState(const ::bosdyn::api::RobotState& robot_state) {
  spot_msgs::msg::FootStateArray foot_states;

  for (const auto& foot : robot_state.foot_state()) {
    spot_msgs::msg::FootState foot_state;
    foot_state.foot_position_rt_body.x = foot.foot_position_rt_body().x();
    foot_state.foot_position_rt_body.y = foot.foot_position_rt_body().y();
    foot_state.foot_position_rt_body.z = foot.foot_position_rt_body().z();
    foot_state.contact = foot.contact();
    foot_states.states.push_back(foot_state);
  }

  return foot_states;
}

spot_msgs::msg::EStopStateArray getEstopStates(const ::bosdyn::api::RobotState& robot_state,
                                               const google::protobuf::Duration& clock_skew) {
  spot_msgs::msg::EStopStateArray estop_states;

  for (const auto& estop : robot_state.estop_states()) {
    spot_msgs::msg::EStopState estop_state;
    estop_state.header.stamp = applyClockSkew(estop.timestamp(), clock_skew);
    estop_state.name = estop.name();
    estop_state.type = estop.type();
    estop_state.state = estop.state();
    estop_state.state_description = estop.state_description();
    estop_states.estop_states.push_back(estop_state);
  }

  return estop_states;
}

std::optional<sensor_msgs::msg::JointState> getJointStates(const ::bosdyn::api::RobotState& robot_state,
                                                           const google::protobuf::Duration& clock_skew,
                                                           const std::string& prefix) {
  if (robot_state.has_kinematic_state()) {
    sensor_msgs::msg::JointState joint_states;
    joint_states.header.stamp = applyClockSkew(robot_state.kinematic_state().acquisition_timestamp(), clock_skew);

    for (const auto& joint : robot_state.kinematic_state().joint_states()) {
      const auto joint_name = prefix + kFriendlyJointNames.at(joint.name());
      joint_states.name.push_back(joint_name);
      joint_states.position.push_back(joint.position().value());
      joint_states.velocity.push_back(joint.velocity().value());
      joint_states.effort.push_back(joint.load().value());
    }

    return joint_states;
  }
  return std::nullopt;
}

std::optional<tf2_msgs::msg::TFMessage> getTf(const ::bosdyn::api::RobotState& robot_state,
                                              const google::protobuf::Duration& clock_skew, const std::string& prefix,
                                              const std::string& inverse_target_frame_id) {
  if (robot_state.has_kinematic_state()) {
    tf2_msgs::msg::TFMessage tf_msg;

    const auto local_time = applyClockSkew(robot_state.kinematic_state().acquisition_timestamp(), clock_skew);

    for (const auto& [frame_id, transform] :
         robot_state.kinematic_state().transforms_snapshot().child_to_parent_edge_map()) {
      // Do not publish frames without parents
      if (transform.parent_frame_name().empty()) {
        continue;
      }
      const auto parent_frame_name = transform.parent_frame_name().find("/") == std::string::npos
                                         ? prefix + transform.parent_frame_name()
                                         : transform.parent_frame_name();
      const auto frame_name = frame_id.find("/") == std::string::npos ? prefix + frame_id : frame_id;

      // set target frame(preferred odom frame) as the root node in tf tree
      if (inverse_target_frame_id == frame_name) {
        tf_msg.transforms.push_back(conversions::toTransformStamped(~(transform.parent_tform_child()), frame_name,
                                                                    parent_frame_name, local_time));
      } else {
        tf_msg.transforms.push_back(
            conversions::toTransformStamped(transform.parent_tform_child(), parent_frame_name, frame_name, local_time));
      }
    }
    return tf_msg;
  }

  return std::nullopt;
}

std::optional<geometry_msgs::msg::TwistWithCovarianceStamped> getOdomTwist(
    const ::bosdyn::api::RobotState& robot_state, const google::protobuf::Duration& clock_skew) {
  if (robot_state.has_kinematic_state() && robot_state.kinematic_state().has_velocity_of_body_in_odom()) {
    geometry_msgs::msg::TwistWithCovarianceStamped odom_twist_msg;
    // TODO(schornakj): need to add the frame ID here?
    odom_twist_msg.header.stamp =
        spot_ros2::applyClockSkew(robot_state.kinematic_state().acquisition_timestamp(), clock_skew);
    const auto& body_velocity = robot_state.kinematic_state().velocity_of_body_in_odom();
    odom_twist_msg.twist.twist.linear.x = body_velocity.linear().x();
    odom_twist_msg.twist.twist.linear.y = body_velocity.linear().y();
    odom_twist_msg.twist.twist.linear.z = body_velocity.linear().z();
    odom_twist_msg.twist.twist.angular.x = body_velocity.angular().x();
    odom_twist_msg.twist.twist.angular.y = body_velocity.angular().y();
    odom_twist_msg.twist.twist.angular.z = body_velocity.angular().z();

    return odom_twist_msg;
  }
  return std::nullopt;
}

std::optional<nav_msgs::msg::Odometry> getOdom(const ::bosdyn::api::RobotState& robot_state,
                                               const google::protobuf::Duration& clock_skew, const std::string& prefix,
                                               bool is_using_vision) {
  if (robot_state.has_kinematic_state() && robot_state.kinematic_state().has_acquisition_timestamp() &&
      robot_state.kinematic_state().has_transforms_snapshot() &&
      robot_state.kinematic_state().has_velocity_of_body_in_odom()) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = applyClockSkew(robot_state.kinematic_state().acquisition_timestamp(), clock_skew);

    ::bosdyn::api::SE3Pose tf_body_pose;
    if (is_using_vision) {
      odom_msg.header.frame_id = prefix + "vision";
      if (!::bosdyn::api::GetWorldTformBody(robot_state.kinematic_state().transforms_snapshot(), &tf_body_pose)) {
        return std::nullopt;
      }
    } else {
      odom_msg.header.frame_id = prefix + "odom";
      if (!::bosdyn::api::GetOdomTformBody(robot_state.kinematic_state().transforms_snapshot(), &tf_body_pose)) {
        return std::nullopt;
      }
    }
    odom_msg.child_frame_id = prefix + "body";

    geometry_msgs::msg::PoseWithCovariance pose_odom_msg;
    pose_odom_msg.pose.position.x = tf_body_pose.position().x();
    pose_odom_msg.pose.position.y = tf_body_pose.position().y();
    pose_odom_msg.pose.position.z = tf_body_pose.position().z();
    pose_odom_msg.pose.orientation.x = tf_body_pose.rotation().x();
    pose_odom_msg.pose.orientation.y = tf_body_pose.rotation().y();
    pose_odom_msg.pose.orientation.z = tf_body_pose.rotation().z();
    pose_odom_msg.pose.orientation.w = tf_body_pose.rotation().w();

    odom_msg.pose = pose_odom_msg;
    odom_msg.twist = getOdomTwist(robot_state, clock_skew).value().twist;
    return odom_msg;
  }
  return std::nullopt;
}

std::optional<spot_msgs::msg::PowerState> getPowerState(const ::bosdyn::api::RobotState& robot_state,
                                                        const google::protobuf::Duration& clock_skew) {
  if (robot_state.has_power_state()) {
    spot_msgs::msg::PowerState power_state;

    power_state.header.stamp = applyClockSkew(robot_state.power_state().timestamp(), clock_skew);
    power_state.motor_power_state = robot_state.power_state().motor_power_state();
    power_state.shore_power_state = robot_state.power_state().shore_power_state();
    power_state.locomotion_charge_percentage = robot_state.power_state().locomotion_charge_percentage().value();
    power_state.locomotion_estimated_runtime =
        builtin_interfaces::build<builtin_interfaces::msg::Duration>()
            .sec(robot_state.power_state().locomotion_estimated_runtime().seconds())
            .nanosec(robot_state.power_state().locomotion_estimated_runtime().nanos());

    return power_state;
  }

  return std::nullopt;
}

std::optional<spot_msgs::msg::SystemFaultState> getSystemFaultState(const ::bosdyn::api::RobotState& robot_state,
                                                                    const google::protobuf::Duration& clock_skew) {
  if (robot_state.has_system_fault_state()) {
    const auto create_fault_message = [&clock_skew](const ::bosdyn::api::SystemFault& fault) {
      spot_msgs::msg::SystemFault fault_msg;
      fault_msg.name = fault.name();
      fault_msg.header.stamp = applyClockSkew(fault.onset_timestamp(), clock_skew);
      fault_msg.duration = builtin_interfaces::build<builtin_interfaces::msg::Duration>()
                               .sec(fault.duration().seconds())
                               .nanosec(fault.duration().nanos());
      fault_msg.code = fault.code();
      fault_msg.uid = fault.uid();
      fault_msg.error_message = fault.error_message();
      for (const auto& attr : fault.attributes()) {
        fault_msg.attributes.push_back(attr);
      }
      fault_msg.severity = fault.severity();
      return fault_msg;
    };

    spot_msgs::msg::SystemFaultState system_fault_state;
    for (const auto& fault : robot_state.system_fault_state().faults()) {
      system_fault_state.faults.push_back(create_fault_message(fault));
    }
    for (const auto& fault : robot_state.system_fault_state().historical_faults()) {
      system_fault_state.historical_faults.push_back(create_fault_message(fault));
    }
    return system_fault_state;
  }
  return std::nullopt;
}
std::optional<bosdyn_msgs::msg::ManipulatorState> getManipulatorState(const ::bosdyn::api::RobotState& robot_state) {
  if (robot_state.has_manipulator_state()) {
    bosdyn_msgs::msg::ManipulatorState manipulator_state;

    manipulator_state.gripper_open_percentage = robot_state.manipulator_state().gripper_open_percentage();
    manipulator_state.is_gripper_holding_item = robot_state.manipulator_state().is_gripper_holding_item();

    const auto& force_in_hand = robot_state.manipulator_state().estimated_end_effector_force_in_hand();
    manipulator_state.estimated_end_effector_force_in_hand.x = force_in_hand.x();
    manipulator_state.estimated_end_effector_force_in_hand.y = force_in_hand.y();
    manipulator_state.estimated_end_effector_force_in_hand.z = force_in_hand.z();
    manipulator_state.estimated_end_effector_force_in_hand_is_set =
        robot_state.manipulator_state().has_estimated_end_effector_force_in_hand();

    manipulator_state.stow_state.value = robot_state.manipulator_state().stow_state();

    const auto& velocity_of_hand_in_vision = robot_state.manipulator_state().velocity_of_hand_in_vision();
    manipulator_state.velocity_of_hand_in_vision.linear.x = velocity_of_hand_in_vision.linear().x();
    manipulator_state.velocity_of_hand_in_vision.linear.y = velocity_of_hand_in_vision.linear().y();
    manipulator_state.velocity_of_hand_in_vision.linear.z = velocity_of_hand_in_vision.linear().z();
    manipulator_state.velocity_of_hand_in_vision.angular.x = velocity_of_hand_in_vision.angular().x();
    manipulator_state.velocity_of_hand_in_vision.angular.y = velocity_of_hand_in_vision.angular().y();
    manipulator_state.velocity_of_hand_in_vision.angular.z = velocity_of_hand_in_vision.angular().z();
    manipulator_state.velocity_of_hand_in_vision_is_set =
        robot_state.manipulator_state().has_velocity_of_hand_in_vision();

    const auto& velocity_of_hand_in_odom = robot_state.manipulator_state().velocity_of_hand_in_odom();
    manipulator_state.velocity_of_hand_in_odom.linear.x = velocity_of_hand_in_odom.linear().x();
    manipulator_state.velocity_of_hand_in_odom.linear.y = velocity_of_hand_in_odom.linear().y();
    manipulator_state.velocity_of_hand_in_odom.linear.z = velocity_of_hand_in_odom.linear().z();
    manipulator_state.velocity_of_hand_in_odom.angular.x = velocity_of_hand_in_odom.angular().x();
    manipulator_state.velocity_of_hand_in_odom.angular.y = velocity_of_hand_in_odom.angular().y();
    manipulator_state.velocity_of_hand_in_odom.angular.z = velocity_of_hand_in_odom.angular().z();
    manipulator_state.velocity_of_hand_in_odom_is_set = robot_state.manipulator_state().has_velocity_of_hand_in_odom();

    manipulator_state.carry_state.value = robot_state.manipulator_state().carry_state();
    return manipulator_state;
  }
  return std::nullopt;
}
std::optional<geometry_msgs::msg::Vector3Stamped> getEndEffectorForce(const ::bosdyn::api::RobotState& robot_state,
                                                                      const google::protobuf::Duration& clock_skew,
                                                                      const std::string& prefix) {
  if (robot_state.has_manipulator_state()) {
    geometry_msgs::msg::Vector3Stamped force;
    force.header.stamp = applyClockSkew(robot_state.kinematic_state().acquisition_timestamp(), clock_skew);
    force.header.frame_id = prefix + "hand";
    force.vector.x = robot_state.manipulator_state().estimated_end_effector_force_in_hand().x();
    force.vector.y = robot_state.manipulator_state().estimated_end_effector_force_in_hand().y();
    force.vector.z = robot_state.manipulator_state().estimated_end_effector_force_in_hand().z();

    return force;
  }
  return std::nullopt;
}
std::optional<spot_msgs::msg::BehaviorFaultState> getBehaviorFaultState(const ::bosdyn::api::RobotState& robot_state,
                                                                        const google::protobuf::Duration& clock_skew) {
  if (robot_state.has_behavior_fault_state()) {
    spot_msgs::msg::BehaviorFaultState behavior_fault_msgs;

    for (const auto& fault : robot_state.behavior_fault_state().faults()) {
      spot_msgs::msg::BehaviorFault fault_msg;
      fault_msg.behavior_fault_id = fault.behavior_fault_id();
      fault_msg.header.stamp = applyClockSkew(fault.onset_timestamp(), clock_skew);
      fault_msg.cause = fault.cause();
      fault_msg.status = fault.status();
      behavior_fault_msgs.faults.push_back(fault_msg);
    }
    return behavior_fault_msgs;
  }
  return std::nullopt;
}

}  // namespace spot_ros2
