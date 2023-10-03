// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from spot_msgs:msg/Feedback.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__FEEDBACK__TRAITS_HPP_
#define SPOT_MSGS__MSG__DETAIL__FEEDBACK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "spot_msgs/msg/detail/feedback__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace spot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: standing
  {
    out << "standing: ";
    rosidl_generator_traits::value_to_yaml(msg.standing, out);
    out << ", ";
  }

  // member: sitting
  {
    out << "sitting: ";
    rosidl_generator_traits::value_to_yaml(msg.sitting, out);
    out << ", ";
  }

  // member: moving
  {
    out << "moving: ";
    rosidl_generator_traits::value_to_yaml(msg.moving, out);
    out << ", ";
  }

  // member: serial_number
  {
    out << "serial_number: ";
    rosidl_generator_traits::value_to_yaml(msg.serial_number, out);
    out << ", ";
  }

  // member: species
  {
    out << "species: ";
    rosidl_generator_traits::value_to_yaml(msg.species, out);
    out << ", ";
  }

  // member: version
  {
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << ", ";
  }

  // member: nickname
  {
    out << "nickname: ";
    rosidl_generator_traits::value_to_yaml(msg.nickname, out);
    out << ", ";
  }

  // member: computer_serial_number
  {
    out << "computer_serial_number: ";
    rosidl_generator_traits::value_to_yaml(msg.computer_serial_number, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: standing
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "standing: ";
    rosidl_generator_traits::value_to_yaml(msg.standing, out);
    out << "\n";
  }

  // member: sitting
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sitting: ";
    rosidl_generator_traits::value_to_yaml(msg.sitting, out);
    out << "\n";
  }

  // member: moving
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "moving: ";
    rosidl_generator_traits::value_to_yaml(msg.moving, out);
    out << "\n";
  }

  // member: serial_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "serial_number: ";
    rosidl_generator_traits::value_to_yaml(msg.serial_number, out);
    out << "\n";
  }

  // member: species
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "species: ";
    rosidl_generator_traits::value_to_yaml(msg.species, out);
    out << "\n";
  }

  // member: version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << "\n";
  }

  // member: nickname
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nickname: ";
    rosidl_generator_traits::value_to_yaml(msg.nickname, out);
    out << "\n";
  }

  // member: computer_serial_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "computer_serial_number: ";
    rosidl_generator_traits::value_to_yaml(msg.computer_serial_number, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Feedback & msg, bool use_flow_style = false)
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
  const spot_msgs::msg::Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  spot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use spot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const spot_msgs::msg::Feedback & msg)
{
  return spot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<spot_msgs::msg::Feedback>()
{
  return "spot_msgs::msg::Feedback";
}

template<>
inline const char * name<spot_msgs::msg::Feedback>()
{
  return "spot_msgs/msg/Feedback";
}

template<>
struct has_fixed_size<spot_msgs::msg::Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<spot_msgs::msg::Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<spot_msgs::msg::Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SPOT_MSGS__MSG__DETAIL__FEEDBACK__TRAITS_HPP_
