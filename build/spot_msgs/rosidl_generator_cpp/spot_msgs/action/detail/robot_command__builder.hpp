// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:action/RobotCommand.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__ACTION__DETAIL__ROBOT_COMMAND__BUILDER_HPP_
#define SPOT_MSGS__ACTION__DETAIL__ROBOT_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/action/detail/robot_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_RobotCommand_Goal_command
{
public:
  Init_RobotCommand_Goal_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::action::RobotCommand_Goal command(::spot_msgs::action::RobotCommand_Goal::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::RobotCommand_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::RobotCommand_Goal>()
{
  return spot_msgs::action::builder::Init_RobotCommand_Goal_command();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_RobotCommand_Result_message
{
public:
  explicit Init_RobotCommand_Result_message(::spot_msgs::action::RobotCommand_Result & msg)
  : msg_(msg)
  {}
  ::spot_msgs::action::RobotCommand_Result message(::spot_msgs::action::RobotCommand_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::RobotCommand_Result msg_;
};

class Init_RobotCommand_Result_success
{
public:
  explicit Init_RobotCommand_Result_success(::spot_msgs::action::RobotCommand_Result & msg)
  : msg_(msg)
  {}
  Init_RobotCommand_Result_message success(::spot_msgs::action::RobotCommand_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_RobotCommand_Result_message(msg_);
  }

private:
  ::spot_msgs::action::RobotCommand_Result msg_;
};

class Init_RobotCommand_Result_result
{
public:
  Init_RobotCommand_Result_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotCommand_Result_success result(::spot_msgs::action::RobotCommand_Result::_result_type arg)
  {
    msg_.result = std::move(arg);
    return Init_RobotCommand_Result_success(msg_);
  }

private:
  ::spot_msgs::action::RobotCommand_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::RobotCommand_Result>()
{
  return spot_msgs::action::builder::Init_RobotCommand_Result_result();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_RobotCommand_Feedback_feedback
{
public:
  Init_RobotCommand_Feedback_feedback()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::action::RobotCommand_Feedback feedback(::spot_msgs::action::RobotCommand_Feedback::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::RobotCommand_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::RobotCommand_Feedback>()
{
  return spot_msgs::action::builder::Init_RobotCommand_Feedback_feedback();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_RobotCommand_SendGoal_Request_goal
{
public:
  explicit Init_RobotCommand_SendGoal_Request_goal(::spot_msgs::action::RobotCommand_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::spot_msgs::action::RobotCommand_SendGoal_Request goal(::spot_msgs::action::RobotCommand_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::RobotCommand_SendGoal_Request msg_;
};

class Init_RobotCommand_SendGoal_Request_goal_id
{
public:
  Init_RobotCommand_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotCommand_SendGoal_Request_goal goal_id(::spot_msgs::action::RobotCommand_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_RobotCommand_SendGoal_Request_goal(msg_);
  }

private:
  ::spot_msgs::action::RobotCommand_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::RobotCommand_SendGoal_Request>()
{
  return spot_msgs::action::builder::Init_RobotCommand_SendGoal_Request_goal_id();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_RobotCommand_SendGoal_Response_stamp
{
public:
  explicit Init_RobotCommand_SendGoal_Response_stamp(::spot_msgs::action::RobotCommand_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::spot_msgs::action::RobotCommand_SendGoal_Response stamp(::spot_msgs::action::RobotCommand_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::RobotCommand_SendGoal_Response msg_;
};

class Init_RobotCommand_SendGoal_Response_accepted
{
public:
  Init_RobotCommand_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotCommand_SendGoal_Response_stamp accepted(::spot_msgs::action::RobotCommand_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_RobotCommand_SendGoal_Response_stamp(msg_);
  }

private:
  ::spot_msgs::action::RobotCommand_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::RobotCommand_SendGoal_Response>()
{
  return spot_msgs::action::builder::Init_RobotCommand_SendGoal_Response_accepted();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_RobotCommand_GetResult_Request_goal_id
{
public:
  Init_RobotCommand_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::action::RobotCommand_GetResult_Request goal_id(::spot_msgs::action::RobotCommand_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::RobotCommand_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::RobotCommand_GetResult_Request>()
{
  return spot_msgs::action::builder::Init_RobotCommand_GetResult_Request_goal_id();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_RobotCommand_GetResult_Response_result
{
public:
  explicit Init_RobotCommand_GetResult_Response_result(::spot_msgs::action::RobotCommand_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::spot_msgs::action::RobotCommand_GetResult_Response result(::spot_msgs::action::RobotCommand_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::RobotCommand_GetResult_Response msg_;
};

class Init_RobotCommand_GetResult_Response_status
{
public:
  Init_RobotCommand_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotCommand_GetResult_Response_result status(::spot_msgs::action::RobotCommand_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_RobotCommand_GetResult_Response_result(msg_);
  }

private:
  ::spot_msgs::action::RobotCommand_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::RobotCommand_GetResult_Response>()
{
  return spot_msgs::action::builder::Init_RobotCommand_GetResult_Response_status();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_RobotCommand_FeedbackMessage_feedback
{
public:
  explicit Init_RobotCommand_FeedbackMessage_feedback(::spot_msgs::action::RobotCommand_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::spot_msgs::action::RobotCommand_FeedbackMessage feedback(::spot_msgs::action::RobotCommand_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::RobotCommand_FeedbackMessage msg_;
};

class Init_RobotCommand_FeedbackMessage_goal_id
{
public:
  Init_RobotCommand_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotCommand_FeedbackMessage_feedback goal_id(::spot_msgs::action::RobotCommand_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_RobotCommand_FeedbackMessage_feedback(msg_);
  }

private:
  ::spot_msgs::action::RobotCommand_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::RobotCommand_FeedbackMessage>()
{
  return spot_msgs::action::builder::Init_RobotCommand_FeedbackMessage_goal_id();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__ACTION__DETAIL__ROBOT_COMMAND__BUILDER_HPP_
