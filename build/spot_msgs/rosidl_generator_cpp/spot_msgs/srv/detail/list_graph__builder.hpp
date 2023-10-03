// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:srv/ListGraph.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__LIST_GRAPH__BUILDER_HPP_
#define SPOT_MSGS__SRV__DETAIL__LIST_GRAPH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/srv/detail/list_graph__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_ListGraph_Request_upload_filepath
{
public:
  Init_ListGraph_Request_upload_filepath()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::srv::ListGraph_Request upload_filepath(::spot_msgs::srv::ListGraph_Request::_upload_filepath_type arg)
  {
    msg_.upload_filepath = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::ListGraph_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::ListGraph_Request>()
{
  return spot_msgs::srv::builder::Init_ListGraph_Request_upload_filepath();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_ListGraph_Response_waypoint_ids
{
public:
  Init_ListGraph_Response_waypoint_ids()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::srv::ListGraph_Response waypoint_ids(::spot_msgs::srv::ListGraph_Response::_waypoint_ids_type arg)
  {
    msg_.waypoint_ids = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::ListGraph_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::ListGraph_Response>()
{
  return spot_msgs::srv::builder::Init_ListGraph_Response_waypoint_ids();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__SRV__DETAIL__LIST_GRAPH__BUILDER_HPP_
