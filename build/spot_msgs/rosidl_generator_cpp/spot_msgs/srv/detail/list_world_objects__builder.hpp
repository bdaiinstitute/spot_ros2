// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:srv/ListWorldObjects.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__LIST_WORLD_OBJECTS__BUILDER_HPP_
#define SPOT_MSGS__SRV__DETAIL__LIST_WORLD_OBJECTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/srv/detail/list_world_objects__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_ListWorldObjects_Request_request
{
public:
  Init_ListWorldObjects_Request_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::srv::ListWorldObjects_Request request(::spot_msgs::srv::ListWorldObjects_Request::_request_type arg)
  {
    msg_.request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::ListWorldObjects_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::ListWorldObjects_Request>()
{
  return spot_msgs::srv::builder::Init_ListWorldObjects_Request_request();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_ListWorldObjects_Response_response
{
public:
  Init_ListWorldObjects_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::srv::ListWorldObjects_Response response(::spot_msgs::srv::ListWorldObjects_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::ListWorldObjects_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::ListWorldObjects_Response>()
{
  return spot_msgs::srv::builder::Init_ListWorldObjects_Response_response();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__SRV__DETAIL__LIST_WORLD_OBJECTS__BUILDER_HPP_
