// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:msg/Metrics.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__METRICS__BUILDER_HPP_
#define SPOT_MSGS__MSG__DETAIL__METRICS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/msg/detail/metrics__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace msg
{

namespace builder
{

class Init_Metrics_electric_power
{
public:
  explicit Init_Metrics_electric_power(::spot_msgs::msg::Metrics & msg)
  : msg_(msg)
  {}
  ::spot_msgs::msg::Metrics electric_power(::spot_msgs::msg::Metrics::_electric_power_type arg)
  {
    msg_.electric_power = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::msg::Metrics msg_;
};

class Init_Metrics_time_moving
{
public:
  explicit Init_Metrics_time_moving(::spot_msgs::msg::Metrics & msg)
  : msg_(msg)
  {}
  Init_Metrics_electric_power time_moving(::spot_msgs::msg::Metrics::_time_moving_type arg)
  {
    msg_.time_moving = std::move(arg);
    return Init_Metrics_electric_power(msg_);
  }

private:
  ::spot_msgs::msg::Metrics msg_;
};

class Init_Metrics_gait_cycles
{
public:
  explicit Init_Metrics_gait_cycles(::spot_msgs::msg::Metrics & msg)
  : msg_(msg)
  {}
  Init_Metrics_time_moving gait_cycles(::spot_msgs::msg::Metrics::_gait_cycles_type arg)
  {
    msg_.gait_cycles = std::move(arg);
    return Init_Metrics_time_moving(msg_);
  }

private:
  ::spot_msgs::msg::Metrics msg_;
};

class Init_Metrics_distance
{
public:
  explicit Init_Metrics_distance(::spot_msgs::msg::Metrics & msg)
  : msg_(msg)
  {}
  Init_Metrics_gait_cycles distance(::spot_msgs::msg::Metrics::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_Metrics_gait_cycles(msg_);
  }

private:
  ::spot_msgs::msg::Metrics msg_;
};

class Init_Metrics_header
{
public:
  Init_Metrics_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Metrics_distance header(::spot_msgs::msg::Metrics::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Metrics_distance(msg_);
  }

private:
  ::spot_msgs::msg::Metrics msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::msg::Metrics>()
{
  return spot_msgs::msg::builder::Init_Metrics_header();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__METRICS__BUILDER_HPP_
