// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from spot_msgs:msg/MobilityParams.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__MOBILITY_PARAMS__TRAITS_HPP_
#define SPOT_MSGS__MSG__DETAIL__MOBILITY_PARAMS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "spot_msgs/msg/detail/mobility_params__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'body_control'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace spot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MobilityParams & msg,
  std::ostream & out)
{
  out << "{";
  // member: body_control
  {
    out << "body_control: ";
    to_flow_style_yaml(msg.body_control, out);
    out << ", ";
  }

  // member: locomotion_hint
  {
    out << "locomotion_hint: ";
    rosidl_generator_traits::value_to_yaml(msg.locomotion_hint, out);
    out << ", ";
  }

  // member: stair_hint
  {
    out << "stair_hint: ";
    rosidl_generator_traits::value_to_yaml(msg.stair_hint, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MobilityParams & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: body_control
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "body_control:\n";
    to_block_style_yaml(msg.body_control, out, indentation + 2);
  }

  // member: locomotion_hint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "locomotion_hint: ";
    rosidl_generator_traits::value_to_yaml(msg.locomotion_hint, out);
    out << "\n";
  }

  // member: stair_hint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stair_hint: ";
    rosidl_generator_traits::value_to_yaml(msg.stair_hint, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MobilityParams & msg, bool use_flow_style = false)
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
  const spot_msgs::msg::MobilityParams & msg,
  std::ostream & out, size_t indentation = 0)
{
  spot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use spot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const spot_msgs::msg::MobilityParams & msg)
{
  return spot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<spot_msgs::msg::MobilityParams>()
{
  return "spot_msgs::msg::MobilityParams";
}

template<>
inline const char * name<spot_msgs::msg::MobilityParams>()
{
  return "spot_msgs/msg/MobilityParams";
}

template<>
struct has_fixed_size<spot_msgs::msg::MobilityParams>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<spot_msgs::msg::MobilityParams>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<spot_msgs::msg::MobilityParams>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SPOT_MSGS__MSG__DETAIL__MOBILITY_PARAMS__TRAITS_HPP_
