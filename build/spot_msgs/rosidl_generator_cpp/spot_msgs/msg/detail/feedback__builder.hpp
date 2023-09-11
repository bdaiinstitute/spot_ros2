// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:msg/Feedback.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__FEEDBACK__BUILDER_HPP_
#define SPOT_MSGS__MSG__DETAIL__FEEDBACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/msg/detail/feedback__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace msg
{

namespace builder
{

class Init_Feedback_computer_serial_number
{
public:
  explicit Init_Feedback_computer_serial_number(::spot_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  ::spot_msgs::msg::Feedback computer_serial_number(::spot_msgs::msg::Feedback::_computer_serial_number_type arg)
  {
    msg_.computer_serial_number = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::msg::Feedback msg_;
};

class Init_Feedback_nickname
{
public:
  explicit Init_Feedback_nickname(::spot_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_computer_serial_number nickname(::spot_msgs::msg::Feedback::_nickname_type arg)
  {
    msg_.nickname = std::move(arg);
    return Init_Feedback_computer_serial_number(msg_);
  }

private:
  ::spot_msgs::msg::Feedback msg_;
};

class Init_Feedback_version
{
public:
  explicit Init_Feedback_version(::spot_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_nickname version(::spot_msgs::msg::Feedback::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_Feedback_nickname(msg_);
  }

private:
  ::spot_msgs::msg::Feedback msg_;
};

class Init_Feedback_species
{
public:
  explicit Init_Feedback_species(::spot_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_version species(::spot_msgs::msg::Feedback::_species_type arg)
  {
    msg_.species = std::move(arg);
    return Init_Feedback_version(msg_);
  }

private:
  ::spot_msgs::msg::Feedback msg_;
};

class Init_Feedback_serial_number
{
public:
  explicit Init_Feedback_serial_number(::spot_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_species serial_number(::spot_msgs::msg::Feedback::_serial_number_type arg)
  {
    msg_.serial_number = std::move(arg);
    return Init_Feedback_species(msg_);
  }

private:
  ::spot_msgs::msg::Feedback msg_;
};

class Init_Feedback_moving
{
public:
  explicit Init_Feedback_moving(::spot_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_serial_number moving(::spot_msgs::msg::Feedback::_moving_type arg)
  {
    msg_.moving = std::move(arg);
    return Init_Feedback_serial_number(msg_);
  }

private:
  ::spot_msgs::msg::Feedback msg_;
};

class Init_Feedback_sitting
{
public:
  explicit Init_Feedback_sitting(::spot_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_moving sitting(::spot_msgs::msg::Feedback::_sitting_type arg)
  {
    msg_.sitting = std::move(arg);
    return Init_Feedback_moving(msg_);
  }

private:
  ::spot_msgs::msg::Feedback msg_;
};

class Init_Feedback_standing
{
public:
  Init_Feedback_standing()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Feedback_sitting standing(::spot_msgs::msg::Feedback::_standing_type arg)
  {
    msg_.standing = std::move(arg);
    return Init_Feedback_sitting(msg_);
  }

private:
  ::spot_msgs::msg::Feedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::msg::Feedback>()
{
  return spot_msgs::msg::builder::Init_Feedback_standing();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__FEEDBACK__BUILDER_HPP_
