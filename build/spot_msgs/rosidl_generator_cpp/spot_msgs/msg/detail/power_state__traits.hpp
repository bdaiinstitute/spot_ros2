// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from spot_msgs:msg/PowerState.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__POWER_STATE__TRAITS_HPP_
#define SPOT_MSGS__MSG__DETAIL__POWER_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "spot_msgs/msg/detail/power_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'locomotion_estimated_runtime'
#include "builtin_interfaces/msg/detail/duration__traits.hpp"

namespace spot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PowerState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: motor_power_state
  {
    out << "motor_power_state: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_power_state, out);
    out << ", ";
  }

  // member: shore_power_state
  {
    out << "shore_power_state: ";
    rosidl_generator_traits::value_to_yaml(msg.shore_power_state, out);
    out << ", ";
  }

  // member: locomotion_charge_percentage
  {
    out << "locomotion_charge_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.locomotion_charge_percentage, out);
    out << ", ";
  }

  // member: locomotion_estimated_runtime
  {
    out << "locomotion_estimated_runtime: ";
    to_flow_style_yaml(msg.locomotion_estimated_runtime, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PowerState & msg,
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

  // member: motor_power_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_power_state: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_power_state, out);
    out << "\n";
  }

  // member: shore_power_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shore_power_state: ";
    rosidl_generator_traits::value_to_yaml(msg.shore_power_state, out);
    out << "\n";
  }

  // member: locomotion_charge_percentage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "locomotion_charge_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.locomotion_charge_percentage, out);
    out << "\n";
  }

  // member: locomotion_estimated_runtime
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "locomotion_estimated_runtime:\n";
    to_block_style_yaml(msg.locomotion_estimated_runtime, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PowerState & msg, bool use_flow_style = false)
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
  const spot_msgs::msg::PowerState & msg,
  std::ostream & out, size_t indentation = 0)
{
  spot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use spot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const spot_msgs::msg::PowerState & msg)
{
  return spot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<spot_msgs::msg::PowerState>()
{
  return "spot_msgs::msg::PowerState";
}

template<>
inline const char * name<spot_msgs::msg::PowerState>()
{
  return "spot_msgs/msg/PowerState";
}

template<>
struct has_fixed_size<spot_msgs::msg::PowerState>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Duration>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<spot_msgs::msg::PowerState>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Duration>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<spot_msgs::msg::PowerState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SPOT_MSGS__MSG__DETAIL__POWER_STATE__TRAITS_HPP_
