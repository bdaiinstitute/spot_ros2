// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from spot_msgs:msg/Lease.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__LEASE__TRAITS_HPP_
#define SPOT_MSGS__MSG__DETAIL__LEASE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "spot_msgs/msg/detail/lease__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace spot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Lease & msg,
  std::ostream & out)
{
  out << "{";
  // member: resource
  {
    out << "resource: ";
    rosidl_generator_traits::value_to_yaml(msg.resource, out);
    out << ", ";
  }

  // member: epoch
  {
    out << "epoch: ";
    rosidl_generator_traits::value_to_yaml(msg.epoch, out);
    out << ", ";
  }

  // member: sequence
  {
    if (msg.sequence.size() == 0) {
      out << "sequence: []";
    } else {
      out << "sequence: [";
      size_t pending_items = msg.sequence.size();
      for (auto item : msg.sequence) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const Lease & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: resource
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "resource: ";
    rosidl_generator_traits::value_to_yaml(msg.resource, out);
    out << "\n";
  }

  // member: epoch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "epoch: ";
    rosidl_generator_traits::value_to_yaml(msg.epoch, out);
    out << "\n";
  }

  // member: sequence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.sequence.size() == 0) {
      out << "sequence: []\n";
    } else {
      out << "sequence:\n";
      for (auto item : msg.sequence) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Lease & msg, bool use_flow_style = false)
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
  const spot_msgs::msg::Lease & msg,
  std::ostream & out, size_t indentation = 0)
{
  spot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use spot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const spot_msgs::msg::Lease & msg)
{
  return spot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<spot_msgs::msg::Lease>()
{
  return "spot_msgs::msg::Lease";
}

template<>
inline const char * name<spot_msgs::msg::Lease>()
{
  return "spot_msgs/msg/Lease";
}

template<>
struct has_fixed_size<spot_msgs::msg::Lease>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<spot_msgs::msg::Lease>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<spot_msgs::msg::Lease>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SPOT_MSGS__MSG__DETAIL__LEASE__TRAITS_HPP_
