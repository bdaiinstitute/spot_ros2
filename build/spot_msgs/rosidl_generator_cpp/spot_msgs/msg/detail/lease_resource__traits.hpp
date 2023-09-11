// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from spot_msgs:msg/LeaseResource.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__LEASE_RESOURCE__TRAITS_HPP_
#define SPOT_MSGS__MSG__DETAIL__LEASE_RESOURCE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "spot_msgs/msg/detail/lease_resource__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'lease'
#include "spot_msgs/msg/detail/lease__traits.hpp"
// Member 'lease_owner'
#include "spot_msgs/msg/detail/lease_owner__traits.hpp"

namespace spot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LeaseResource & msg,
  std::ostream & out)
{
  out << "{";
  // member: resource
  {
    out << "resource: ";
    rosidl_generator_traits::value_to_yaml(msg.resource, out);
    out << ", ";
  }

  // member: lease
  {
    out << "lease: ";
    to_flow_style_yaml(msg.lease, out);
    out << ", ";
  }

  // member: lease_owner
  {
    out << "lease_owner: ";
    to_flow_style_yaml(msg.lease_owner, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LeaseResource & msg,
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

  // member: lease
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lease:\n";
    to_block_style_yaml(msg.lease, out, indentation + 2);
  }

  // member: lease_owner
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lease_owner:\n";
    to_block_style_yaml(msg.lease_owner, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LeaseResource & msg, bool use_flow_style = false)
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
  const spot_msgs::msg::LeaseResource & msg,
  std::ostream & out, size_t indentation = 0)
{
  spot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use spot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const spot_msgs::msg::LeaseResource & msg)
{
  return spot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<spot_msgs::msg::LeaseResource>()
{
  return "spot_msgs::msg::LeaseResource";
}

template<>
inline const char * name<spot_msgs::msg::LeaseResource>()
{
  return "spot_msgs/msg/LeaseResource";
}

template<>
struct has_fixed_size<spot_msgs::msg::LeaseResource>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<spot_msgs::msg::LeaseResource>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<spot_msgs::msg::LeaseResource>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SPOT_MSGS__MSG__DETAIL__LEASE_RESOURCE__TRAITS_HPP_
