// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from spot_msgs:srv/GraphNavClearGraph.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_CLEAR_GRAPH__TRAITS_HPP_
#define SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_CLEAR_GRAPH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "spot_msgs/srv/detail/graph_nav_clear_graph__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace spot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GraphNavClearGraph_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GraphNavClearGraph_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GraphNavClearGraph_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace spot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use spot_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const spot_msgs::srv::GraphNavClearGraph_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  spot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use spot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const spot_msgs::srv::GraphNavClearGraph_Request & msg)
{
  return spot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<spot_msgs::srv::GraphNavClearGraph_Request>()
{
  return "spot_msgs::srv::GraphNavClearGraph_Request";
}

template<>
inline const char * name<spot_msgs::srv::GraphNavClearGraph_Request>()
{
  return "spot_msgs/srv/GraphNavClearGraph_Request";
}

template<>
struct has_fixed_size<spot_msgs::srv::GraphNavClearGraph_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<spot_msgs::srv::GraphNavClearGraph_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<spot_msgs::srv::GraphNavClearGraph_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace spot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GraphNavClearGraph_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GraphNavClearGraph_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GraphNavClearGraph_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace spot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use spot_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const spot_msgs::srv::GraphNavClearGraph_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  spot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use spot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const spot_msgs::srv::GraphNavClearGraph_Response & msg)
{
  return spot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<spot_msgs::srv::GraphNavClearGraph_Response>()
{
  return "spot_msgs::srv::GraphNavClearGraph_Response";
}

template<>
inline const char * name<spot_msgs::srv::GraphNavClearGraph_Response>()
{
  return "spot_msgs/srv/GraphNavClearGraph_Response";
}

template<>
struct has_fixed_size<spot_msgs::srv::GraphNavClearGraph_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<spot_msgs::srv::GraphNavClearGraph_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<spot_msgs::srv::GraphNavClearGraph_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<spot_msgs::srv::GraphNavClearGraph>()
{
  return "spot_msgs::srv::GraphNavClearGraph";
}

template<>
inline const char * name<spot_msgs::srv::GraphNavClearGraph>()
{
  return "spot_msgs/srv/GraphNavClearGraph";
}

template<>
struct has_fixed_size<spot_msgs::srv::GraphNavClearGraph>
  : std::integral_constant<
    bool,
    has_fixed_size<spot_msgs::srv::GraphNavClearGraph_Request>::value &&
    has_fixed_size<spot_msgs::srv::GraphNavClearGraph_Response>::value
  >
{
};

template<>
struct has_bounded_size<spot_msgs::srv::GraphNavClearGraph>
  : std::integral_constant<
    bool,
    has_bounded_size<spot_msgs::srv::GraphNavClearGraph_Request>::value &&
    has_bounded_size<spot_msgs::srv::GraphNavClearGraph_Response>::value
  >
{
};

template<>
struct is_service<spot_msgs::srv::GraphNavClearGraph>
  : std::true_type
{
};

template<>
struct is_service_request<spot_msgs::srv::GraphNavClearGraph_Request>
  : std::true_type
{
};

template<>
struct is_service_response<spot_msgs::srv::GraphNavClearGraph_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_CLEAR_GRAPH__TRAITS_HPP_
