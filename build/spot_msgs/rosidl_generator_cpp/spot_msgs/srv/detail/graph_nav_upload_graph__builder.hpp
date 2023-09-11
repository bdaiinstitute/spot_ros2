// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:srv/GraphNavUploadGraph.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_UPLOAD_GRAPH__BUILDER_HPP_
#define SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_UPLOAD_GRAPH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/srv/detail/graph_nav_upload_graph__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_GraphNavUploadGraph_Request_upload_filepath
{
public:
  Init_GraphNavUploadGraph_Request_upload_filepath()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::srv::GraphNavUploadGraph_Request upload_filepath(::spot_msgs::srv::GraphNavUploadGraph_Request::_upload_filepath_type arg)
  {
    msg_.upload_filepath = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::GraphNavUploadGraph_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::GraphNavUploadGraph_Request>()
{
  return spot_msgs::srv::builder::Init_GraphNavUploadGraph_Request_upload_filepath();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_GraphNavUploadGraph_Response_message
{
public:
  explicit Init_GraphNavUploadGraph_Response_message(::spot_msgs::srv::GraphNavUploadGraph_Response & msg)
  : msg_(msg)
  {}
  ::spot_msgs::srv::GraphNavUploadGraph_Response message(::spot_msgs::srv::GraphNavUploadGraph_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::GraphNavUploadGraph_Response msg_;
};

class Init_GraphNavUploadGraph_Response_success
{
public:
  Init_GraphNavUploadGraph_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GraphNavUploadGraph_Response_message success(::spot_msgs::srv::GraphNavUploadGraph_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GraphNavUploadGraph_Response_message(msg_);
  }

private:
  ::spot_msgs::srv::GraphNavUploadGraph_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::GraphNavUploadGraph_Response>()
{
  return spot_msgs::srv::builder::Init_GraphNavUploadGraph_Response_success();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_UPLOAD_GRAPH__BUILDER_HPP_
