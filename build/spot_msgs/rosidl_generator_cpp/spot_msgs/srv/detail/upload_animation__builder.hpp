// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:srv/UploadAnimation.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__UPLOAD_ANIMATION__BUILDER_HPP_
#define SPOT_MSGS__SRV__DETAIL__UPLOAD_ANIMATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/srv/detail/upload_animation__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_UploadAnimation_Request_animation_file_content
{
public:
  explicit Init_UploadAnimation_Request_animation_file_content(::spot_msgs::srv::UploadAnimation_Request & msg)
  : msg_(msg)
  {}
  ::spot_msgs::srv::UploadAnimation_Request animation_file_content(::spot_msgs::srv::UploadAnimation_Request::_animation_file_content_type arg)
  {
    msg_.animation_file_content = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::UploadAnimation_Request msg_;
};

class Init_UploadAnimation_Request_animation_name
{
public:
  Init_UploadAnimation_Request_animation_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UploadAnimation_Request_animation_file_content animation_name(::spot_msgs::srv::UploadAnimation_Request::_animation_name_type arg)
  {
    msg_.animation_name = std::move(arg);
    return Init_UploadAnimation_Request_animation_file_content(msg_);
  }

private:
  ::spot_msgs::srv::UploadAnimation_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::UploadAnimation_Request>()
{
  return spot_msgs::srv::builder::Init_UploadAnimation_Request_animation_name();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_UploadAnimation_Response_message
{
public:
  explicit Init_UploadAnimation_Response_message(::spot_msgs::srv::UploadAnimation_Response & msg)
  : msg_(msg)
  {}
  ::spot_msgs::srv::UploadAnimation_Response message(::spot_msgs::srv::UploadAnimation_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::UploadAnimation_Response msg_;
};

class Init_UploadAnimation_Response_success
{
public:
  Init_UploadAnimation_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UploadAnimation_Response_message success(::spot_msgs::srv::UploadAnimation_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_UploadAnimation_Response_message(msg_);
  }

private:
  ::spot_msgs::srv::UploadAnimation_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::UploadAnimation_Response>()
{
  return spot_msgs::srv::builder::Init_UploadAnimation_Response_success();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__SRV__DETAIL__UPLOAD_ANIMATION__BUILDER_HPP_
