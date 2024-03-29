// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <google/protobuf/timestamp.pb.h>
#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <spot_driver/api/time_sync_api.hpp>
#include <spot_msgs/msg/battery_state.hpp>
#include <spot_msgs/msg/battery_state_array.hpp>
#include <spot_msgs/msg/e_stop_state.hpp>
#include <spot_msgs/msg/power_state.hpp>
#include <spot_msgs/msg/system_fault.hpp>
#include <spot_msgs/msg/wi_fi_state.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>

// This header defines the << operator for message types which appear as inputs or outputs in this package's unit tests.
// This makes the test output from GMock's matchers much more human-legible than it would be by default.

namespace std {
inline std::ostream& operator<<(std::ostream& os, const google::protobuf::Timestamp& obj) {
  return os << "(google::protobuf::Timestamp: "
            << "seconds: " << obj.seconds() << ", nanos: " << obj.nanos() << ")";
}

inline std::ostream& operator<<(std::ostream& os, const google::protobuf::Duration& obj) {
  return os << "(google::protobuf::Duration: "
            << "seconds: " << obj.seconds() << ", nanos: " << obj.nanos() << ")";
}

inline std::ostream& operator<<(std::ostream& os, const std::vector<std::string>& obj) {
  os << "{";
  for (const auto& entry : obj) {
    os << entry << ", ";
  }
  os << "}";
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const builtin_interfaces::msg::Time& obj) {
  builtin_interfaces::msg::to_flow_style_yaml(obj, os);
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const builtin_interfaces::msg::Duration& obj) {
  builtin_interfaces::msg::to_flow_style_yaml(obj, os);
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const std_msgs::msg::Header& obj) {
  std_msgs::msg::to_flow_style_yaml(obj, os);
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Quaternion& obj) {
  geometry_msgs::msg::to_flow_style_yaml(obj, os);
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Transform& obj) {
  geometry_msgs::msg::to_flow_style_yaml(obj, os);
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::TransformStamped& obj) {
  geometry_msgs::msg::to_flow_style_yaml(obj, os);
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Twist& obj) {
  geometry_msgs::msg::to_flow_style_yaml(obj, os);
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Vector3& obj) {
  geometry_msgs::msg::to_flow_style_yaml(obj, os);
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const spot_msgs::msg::BatteryState& obj) {
  spot_msgs::msg::to_flow_style_yaml(obj, os);
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const spot_msgs::msg::BatteryStateArray& obj) {
  spot_msgs::msg::to_flow_style_yaml(obj, os);
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const spot_msgs::msg::EStopState& obj) {
  spot_msgs::msg::to_flow_style_yaml(obj, os);
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const spot_msgs::msg::PowerState& obj) {
  spot_msgs::msg::to_flow_style_yaml(obj, os);
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const spot_msgs::msg::SystemFault& obj) {
  spot_msgs::msg::to_flow_style_yaml(obj, os);
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const spot_msgs::msg::WiFiState& obj) {
  spot_msgs::msg::to_flow_style_yaml(obj, os);
  return os;
}
}  // namespace std
