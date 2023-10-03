// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from spot_msgs:msg/LeaseOwner.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__LEASE_OWNER__TRAITS_HPP_
#define SPOT_MSGS__MSG__DETAIL__LEASE_OWNER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "spot_msgs/msg/detail/lease_owner__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace spot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LeaseOwner & msg,
  std::ostream & out)
{
  out << "{";
  // member: client_name
  {
    out << "client_name: ";
    rosidl_generator_traits::value_to_yaml(msg.client_name, out);
    out << ", ";
  }

  // member: user_name
  {
    out << "user_name: ";
    rosidl_generator_traits::value_to_yaml(msg.user_name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LeaseOwner & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: client_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "client_name: ";
    rosidl_generator_traits::value_to_yaml(msg.client_name, out);
    out << "\n";
  }

  // member: user_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "user_name: ";
    rosidl_generator_traits::value_to_yaml(msg.user_name, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LeaseOwner & msg, bool use_flow_style = false)
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
  const spot_msgs::msg::LeaseOwner & msg,
  std::ostream & out, size_t indentation = 0)
{
  spot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use spot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const spot_msgs::msg::LeaseOwner & msg)
{
  return spot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<spot_msgs::msg::LeaseOwner>()
{
  return "spot_msgs::msg::LeaseOwner";
}

template<>
inline const char * name<spot_msgs::msg::LeaseOwner>()
{
  return "spot_msgs/msg/LeaseOwner";
}

template<>
struct has_fixed_size<spot_msgs::msg::LeaseOwner>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<spot_msgs::msg::LeaseOwner>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<spot_msgs::msg::LeaseOwner>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SPOT_MSGS__MSG__DETAIL__LEASE_OWNER__TRAITS_HPP_
