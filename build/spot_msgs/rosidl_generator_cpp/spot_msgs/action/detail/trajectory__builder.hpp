// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:action/Trajectory.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__ACTION__DETAIL__TRAJECTORY__BUILDER_HPP_
#define SPOT_MSGS__ACTION__DETAIL__TRAJECTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/action/detail/trajectory__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_Trajectory_Goal_precise_positioning
{
public:
  explicit Init_Trajectory_Goal_precise_positioning(::spot_msgs::action::Trajectory_Goal & msg)
  : msg_(msg)
  {}
  ::spot_msgs::action::Trajectory_Goal precise_positioning(::spot_msgs::action::Trajectory_Goal::_precise_positioning_type arg)
  {
    msg_.precise_positioning = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::Trajectory_Goal msg_;
};

class Init_Trajectory_Goal_duration
{
public:
  explicit Init_Trajectory_Goal_duration(::spot_msgs::action::Trajectory_Goal & msg)
  : msg_(msg)
  {}
  Init_Trajectory_Goal_precise_positioning duration(::spot_msgs::action::Trajectory_Goal::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return Init_Trajectory_Goal_precise_positioning(msg_);
  }

private:
  ::spot_msgs::action::Trajectory_Goal msg_;
};

class Init_Trajectory_Goal_target_pose
{
public:
  Init_Trajectory_Goal_target_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Trajectory_Goal_duration target_pose(::spot_msgs::action::Trajectory_Goal::_target_pose_type arg)
  {
    msg_.target_pose = std::move(arg);
    return Init_Trajectory_Goal_duration(msg_);
  }

private:
  ::spot_msgs::action::Trajectory_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::Trajectory_Goal>()
{
  return spot_msgs::action::builder::Init_Trajectory_Goal_target_pose();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_Trajectory_Result_message
{
public:
  explicit Init_Trajectory_Result_message(::spot_msgs::action::Trajectory_Result & msg)
  : msg_(msg)
  {}
  ::spot_msgs::action::Trajectory_Result message(::spot_msgs::action::Trajectory_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::Trajectory_Result msg_;
};

class Init_Trajectory_Result_success
{
public:
  Init_Trajectory_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Trajectory_Result_message success(::spot_msgs::action::Trajectory_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Trajectory_Result_message(msg_);
  }

private:
  ::spot_msgs::action::Trajectory_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::Trajectory_Result>()
{
  return spot_msgs::action::builder::Init_Trajectory_Result_success();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_Trajectory_Feedback_feedback
{
public:
  Init_Trajectory_Feedback_feedback()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::action::Trajectory_Feedback feedback(::spot_msgs::action::Trajectory_Feedback::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::Trajectory_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::Trajectory_Feedback>()
{
  return spot_msgs::action::builder::Init_Trajectory_Feedback_feedback();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_Trajectory_SendGoal_Request_goal
{
public:
  explicit Init_Trajectory_SendGoal_Request_goal(::spot_msgs::action::Trajectory_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::spot_msgs::action::Trajectory_SendGoal_Request goal(::spot_msgs::action::Trajectory_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::Trajectory_SendGoal_Request msg_;
};

class Init_Trajectory_SendGoal_Request_goal_id
{
public:
  Init_Trajectory_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Trajectory_SendGoal_Request_goal goal_id(::spot_msgs::action::Trajectory_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Trajectory_SendGoal_Request_goal(msg_);
  }

private:
  ::spot_msgs::action::Trajectory_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::Trajectory_SendGoal_Request>()
{
  return spot_msgs::action::builder::Init_Trajectory_SendGoal_Request_goal_id();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_Trajectory_SendGoal_Response_stamp
{
public:
  explicit Init_Trajectory_SendGoal_Response_stamp(::spot_msgs::action::Trajectory_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::spot_msgs::action::Trajectory_SendGoal_Response stamp(::spot_msgs::action::Trajectory_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::Trajectory_SendGoal_Response msg_;
};

class Init_Trajectory_SendGoal_Response_accepted
{
public:
  Init_Trajectory_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Trajectory_SendGoal_Response_stamp accepted(::spot_msgs::action::Trajectory_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Trajectory_SendGoal_Response_stamp(msg_);
  }

private:
  ::spot_msgs::action::Trajectory_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::Trajectory_SendGoal_Response>()
{
  return spot_msgs::action::builder::Init_Trajectory_SendGoal_Response_accepted();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_Trajectory_GetResult_Request_goal_id
{
public:
  Init_Trajectory_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::action::Trajectory_GetResult_Request goal_id(::spot_msgs::action::Trajectory_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::Trajectory_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::Trajectory_GetResult_Request>()
{
  return spot_msgs::action::builder::Init_Trajectory_GetResult_Request_goal_id();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_Trajectory_GetResult_Response_result
{
public:
  explicit Init_Trajectory_GetResult_Response_result(::spot_msgs::action::Trajectory_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::spot_msgs::action::Trajectory_GetResult_Response result(::spot_msgs::action::Trajectory_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::Trajectory_GetResult_Response msg_;
};

class Init_Trajectory_GetResult_Response_status
{
public:
  Init_Trajectory_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Trajectory_GetResult_Response_result status(::spot_msgs::action::Trajectory_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Trajectory_GetResult_Response_result(msg_);
  }

private:
  ::spot_msgs::action::Trajectory_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::Trajectory_GetResult_Response>()
{
  return spot_msgs::action::builder::Init_Trajectory_GetResult_Response_status();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_Trajectory_FeedbackMessage_feedback
{
public:
  explicit Init_Trajectory_FeedbackMessage_feedback(::spot_msgs::action::Trajectory_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::spot_msgs::action::Trajectory_FeedbackMessage feedback(::spot_msgs::action::Trajectory_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::Trajectory_FeedbackMessage msg_;
};

class Init_Trajectory_FeedbackMessage_goal_id
{
public:
  Init_Trajectory_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Trajectory_FeedbackMessage_feedback goal_id(::spot_msgs::action::Trajectory_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Trajectory_FeedbackMessage_feedback(msg_);
  }

private:
  ::spot_msgs::action::Trajectory_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::Trajectory_FeedbackMessage>()
{
  return spot_msgs::action::builder::Init_Trajectory_FeedbackMessage_goal_id();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__ACTION__DETAIL__TRAJECTORY__BUILDER_HPP_
