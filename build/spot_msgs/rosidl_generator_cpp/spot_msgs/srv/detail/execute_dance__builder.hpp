// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:srv/ExecuteDance.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__EXECUTE_DANCE__BUILDER_HPP_
#define SPOT_MSGS__SRV__DETAIL__EXECUTE_DANCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/srv/detail/execute_dance__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_ExecuteDance_Request_choreo_file_content
{
public:
  Init_ExecuteDance_Request_choreo_file_content()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::srv::ExecuteDance_Request choreo_file_content(::spot_msgs::srv::ExecuteDance_Request::_choreo_file_content_type arg)
  {
    msg_.choreo_file_content = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::ExecuteDance_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::ExecuteDance_Request>()
{
  return spot_msgs::srv::builder::Init_ExecuteDance_Request_choreo_file_content();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_ExecuteDance_Response_message
{
public:
  explicit Init_ExecuteDance_Response_message(::spot_msgs::srv::ExecuteDance_Response & msg)
  : msg_(msg)
  {}
  ::spot_msgs::srv::ExecuteDance_Response message(::spot_msgs::srv::ExecuteDance_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::ExecuteDance_Response msg_;
};

class Init_ExecuteDance_Response_success
{
public:
  Init_ExecuteDance_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteDance_Response_message success(::spot_msgs::srv::ExecuteDance_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ExecuteDance_Response_message(msg_);
  }

private:
  ::spot_msgs::srv::ExecuteDance_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::ExecuteDance_Response>()
{
  return spot_msgs::srv::builder::Init_ExecuteDance_Response_success();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__SRV__DETAIL__EXECUTE_DANCE__BUILDER_HPP_
