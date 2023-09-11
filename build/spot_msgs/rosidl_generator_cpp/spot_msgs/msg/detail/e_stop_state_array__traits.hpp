// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from spot_msgs:msg/EStopStateArray.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__E_STOP_STATE_ARRAY__TRAITS_HPP_
#define SPOT_MSGS__MSG__DETAIL__E_STOP_STATE_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "spot_msgs/msg/detail/e_stop_state_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'estop_states'
#include "spot_msgs/msg/detail/e_stop_state__traits.hpp"

namespace spot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EStopStateArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: estop_states
  {
    if (msg.estop_states.size() == 0) {
      out << "estop_states: []";
    } else {
      out << "estop_states: [";
      size_t pending_items = msg.estop_states.size();
      for (auto item : msg.estop_states) {
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
  const EStopStateArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: estop_states
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.estop_states.size() == 0) {
      out << "estop_states: []\n";
    } else {
      out << "estop_states:\n";
      for (auto item : msg.estop_states) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EStopStateArray & msg, bool use_flow_style = false)
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
  const spot_msgs::msg::EStopStateArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  spot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use spot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const spot_msgs::msg::EStopStateArray & msg)
{
  return spot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<spot_msgs::msg::EStopStateArray>()
{
  return "spot_msgs::msg::EStopStateArray";
}

template<>
inline const char * name<spot_msgs::msg::EStopStateArray>()
{
  return "spot_msgs/msg/EStopStateArray";
}

template<>
struct has_fixed_size<spot_msgs::msg::EStopStateArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<spot_msgs::msg::EStopStateArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<spot_msgs::msg::EStopStateArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SPOT_MSGS__MSG__DETAIL__E_STOP_STATE_ARRAY__TRAITS_HPP_
