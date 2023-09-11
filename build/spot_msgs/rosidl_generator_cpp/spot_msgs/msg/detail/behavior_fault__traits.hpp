// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from spot_msgs:msg/BehaviorFault.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__BEHAVIOR_FAULT__TRAITS_HPP_
#define SPOT_MSGS__MSG__DETAIL__BEHAVIOR_FAULT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "spot_msgs/msg/detail/behavior_fault__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace spot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BehaviorFault & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: behavior_fault_id
  {
    out << "behavior_fault_id: ";
    rosidl_generator_traits::value_to_yaml(msg.behavior_fault_id, out);
    out << ", ";
  }

  // member: cause
  {
    out << "cause: ";
    rosidl_generator_traits::value_to_yaml(msg.cause, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BehaviorFault & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: behavior_fault_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "behavior_fault_id: ";
    rosidl_generator_traits::value_to_yaml(msg.behavior_fault_id, out);
    out << "\n";
  }

  // member: cause
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cause: ";
    rosidl_generator_traits::value_to_yaml(msg.cause, out);
    out << "\n";
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BehaviorFault & msg, bool use_flow_style = false)
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
  const spot_msgs::msg::BehaviorFault & msg,
  std::ostream & out, size_t indentation = 0)
{
  spot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use spot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const spot_msgs::msg::BehaviorFault & msg)
{
  return spot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<spot_msgs::msg::BehaviorFault>()
{
  return "spot_msgs::msg::BehaviorFault";
}

template<>
inline const char * name<spot_msgs::msg::BehaviorFault>()
{
  return "spot_msgs/msg/BehaviorFault";
}

template<>
struct has_fixed_size<spot_msgs::msg::BehaviorFault>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<spot_msgs::msg::BehaviorFault>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<spot_msgs::msg::BehaviorFault>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SPOT_MSGS__MSG__DETAIL__BEHAVIOR_FAULT__TRAITS_HPP_
