// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from spot_msgs:msg/BehaviorFaultState.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__BEHAVIOR_FAULT_STATE__TRAITS_HPP_
#define SPOT_MSGS__MSG__DETAIL__BEHAVIOR_FAULT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "spot_msgs/msg/detail/behavior_fault_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'faults'
#include "spot_msgs/msg/detail/behavior_fault__traits.hpp"

namespace spot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BehaviorFaultState & msg,
  std::ostream & out)
{
  out << "{";
  // member: faults
  {
    if (msg.faults.size() == 0) {
      out << "faults: []";
    } else {
      out << "faults: [";
      size_t pending_items = msg.faults.size();
      for (auto item : msg.faults) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BehaviorFaultState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: faults
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.faults.size() == 0) {
      out << "faults: []\n";
    } else {
      out << "faults:\n";
      for (auto item : msg.faults) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BehaviorFaultState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace spot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use spot_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const spot_msgs::msg::BehaviorFaultState & msg,
  std::ostream & out, size_t indentation = 0)
{
  spot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use spot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const spot_msgs::msg::BehaviorFaultState & msg)
{
  return spot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<spot_msgs::msg::BehaviorFaultState>()
{
  return "spot_msgs::msg::BehaviorFaultState";
}

template<>
inline const char * name<spot_msgs::msg::BehaviorFaultState>()
{
  return "spot_msgs/msg/BehaviorFaultState";
}

template<>
struct has_fixed_size<spot_msgs::msg::BehaviorFaultState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<spot_msgs::msg::BehaviorFaultState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<spot_msgs::msg::BehaviorFaultState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SPOT_MSGS__MSG__DETAIL__BEHAVIOR_FAULT_STATE__TRAITS_HPP_
