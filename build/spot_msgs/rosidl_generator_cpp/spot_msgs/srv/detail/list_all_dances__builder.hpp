// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:srv/ListAllDances.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__LIST_ALL_DANCES__BUILDER_HPP_
#define SPOT_MSGS__SRV__DETAIL__LIST_ALL_DANCES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/srv/detail/list_all_dances__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::ListAllDances_Request>()
{
  return ::spot_msgs::srv::ListAllDances_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_ListAllDances_Response_dances
{
public:
  explicit Init_ListAllDances_Response_dances(::spot_msgs::srv::ListAllDances_Response & msg)
  : msg_(msg)
  {}
  ::spot_msgs::srv::ListAllDances_Response dances(::spot_msgs::srv::ListAllDances_Response::_dances_type arg)
  {
    msg_.dances = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::ListAllDances_Response msg_;
};

class Init_ListAllDances_Response_message
{
public:
  explicit Init_ListAllDances_Response_message(::spot_msgs::srv::ListAllDances_Response & msg)
  : msg_(msg)
  {}
  Init_ListAllDances_Response_dances message(::spot_msgs::srv::ListAllDances_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_ListAllDances_Response_dances(msg_);
  }

private:
  ::spot_msgs::srv::ListAllDances_Response msg_;
};

class Init_ListAllDances_Response_success
{
public:
  Init_ListAllDances_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ListAllDances_Response_message success(::spot_msgs::srv::ListAllDances_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ListAllDances_Response_message(msg_);
  }

private:
  ::spot_msgs::srv::ListAllDances_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::ListAllDances_Response>()
{
  return spot_msgs::srv::builder::Init_ListAllDances_Response_success();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__SRV__DETAIL__LIST_ALL_DANCES__BUILDER_HPP_
