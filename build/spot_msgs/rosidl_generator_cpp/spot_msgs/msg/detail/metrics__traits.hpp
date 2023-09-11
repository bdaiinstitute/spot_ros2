// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from spot_msgs:msg/Metrics.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__METRICS__TRAITS_HPP_
#define SPOT_MSGS__MSG__DETAIL__METRICS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "spot_msgs/msg/detail/metrics__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'time_moving'
// Member 'electric_power'
#include "builtin_interfaces/msg/detail/duration__traits.hpp"

namespace spot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Metrics & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << ", ";
  }

  // member: gait_cycles
  {
    out << "gait_cycles: ";
    rosidl_generator_traits::value_to_yaml(msg.gait_cycles, out);
    out << ", ";
  }

  // member: time_moving
  {
    out << "time_moving: ";
    to_flow_style_yaml(msg.time_moving, out);
    out << ", ";
  }

  // member: electric_power
  {
    out << "electric_power: ";
    to_flow_style_yaml(msg.electric_power, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Metrics & msg,
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

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }

  // member: gait_cycles
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gait_cycles: ";
    rosidl_generator_traits::value_to_yaml(msg.gait_cycles, out);
    out << "\n";
  }

  // member: time_moving
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_moving:\n";
    to_block_style_yaml(msg.time_moving, out, indentation + 2);
  }

  // member: electric_power
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "electric_power:\n";
    to_block_style_yaml(msg.electric_power, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Metrics & msg, bool use_flow_style = false)
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
  const spot_msgs::msg::Metrics & msg,
  std::ostream & out, size_t indentation = 0)
{
  spot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use spot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const spot_msgs::msg::Metrics & msg)
{
  return spot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<spot_msgs::msg::Metrics>()
{
  return "spot_msgs::msg::Metrics";
}

template<>
inline const char * name<spot_msgs::msg::Metrics>()
{
  return "spot_msgs/msg/Metrics";
}

template<>
struct has_fixed_size<spot_msgs::msg::Metrics>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Duration>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<spot_msgs::msg::Metrics>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Duration>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<spot_msgs::msg::Metrics>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SPOT_MSGS__MSG__DETAIL__METRICS__TRAITS_HPP_
