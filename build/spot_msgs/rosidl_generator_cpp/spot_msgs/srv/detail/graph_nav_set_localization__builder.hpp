// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:srv/GraphNavSetLocalization.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_SET_LOCALIZATION__BUILDER_HPP_
#define SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_SET_LOCALIZATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/srv/detail/graph_nav_set_localization__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_GraphNavSetLocalization_Request_waypoint_id
{
public:
  explicit Init_GraphNavSetLocalization_Request_waypoint_id(::spot_msgs::srv::GraphNavSetLocalization_Request & msg)
  : msg_(msg)
  {}
  ::spot_msgs::srv::GraphNavSetLocalization_Request waypoint_id(::spot_msgs::srv::GraphNavSetLocalization_Request::_waypoint_id_type arg)
  {
    msg_.waypoint_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::GraphNavSetLocalization_Request msg_;
};

class Init_GraphNavSetLocalization_Request_method
{
public:
  Init_GraphNavSetLocalization_Request_method()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GraphNavSetLocalization_Request_waypoint_id method(::spot_msgs::srv::GraphNavSetLocalization_Request::_method_type arg)
  {
    msg_.method = std::move(arg);
    return Init_GraphNavSetLocalization_Request_waypoint_id(msg_);
  }

private:
  ::spot_msgs::srv::GraphNavSetLocalization_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::GraphNavSetLocalization_Request>()
{
  return spot_msgs::srv::builder::Init_GraphNavSetLocalization_Request_method();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_GraphNavSetLocalization_Response_message
{
public:
  explicit Init_GraphNavSetLocalization_Response_message(::spot_msgs::srv::GraphNavSetLocalization_Response & msg)
  : msg_(msg)
  {}
  ::spot_msgs::srv::GraphNavSetLocalization_Response message(::spot_msgs::srv::GraphNavSetLocalization_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::GraphNavSetLocalization_Response msg_;
};

class Init_GraphNavSetLocalization_Response_success
{
public:
  Init_GraphNavSetLocalization_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GraphNavSetLocalization_Response_message success(::spot_msgs::srv::GraphNavSetLocalization_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GraphNavSetLocalization_Response_message(msg_);
  }

private:
  ::spot_msgs::srv::GraphNavSetLocalization_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::GraphNavSetLocalization_Response>()
{
  return spot_msgs::srv::builder::Init_GraphNavSetLocalization_Response_success();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_SET_LOCALIZATION__BUILDER_HPP_
