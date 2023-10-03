// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:srv/Dock.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__DOCK__BUILDER_HPP_
#define SPOT_MSGS__SRV__DETAIL__DOCK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/srv/detail/dock__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_Dock_Request_dock_id
{
public:
  Init_Dock_Request_dock_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::srv::Dock_Request dock_id(::spot_msgs::srv::Dock_Request::_dock_id_type arg)
  {
    msg_.dock_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::Dock_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::Dock_Request>()
{
  return spot_msgs::srv::builder::Init_Dock_Request_dock_id();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_Dock_Response_message
{
public:
  explicit Init_Dock_Response_message(::spot_msgs::srv::Dock_Response & msg)
  : msg_(msg)
  {}
  ::spot_msgs::srv::Dock_Response message(::spot_msgs::srv::Dock_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::Dock_Response msg_;
};

class Init_Dock_Response_success
{
public:
  Init_Dock_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Dock_Response_message success(::spot_msgs::srv::Dock_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Dock_Response_message(msg_);
  }

private:
  ::spot_msgs::srv::Dock_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::Dock_Response>()
{
  return spot_msgs::srv::builder::Init_Dock_Response_success();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__SRV__DETAIL__DOCK__BUILDER_HPP_
