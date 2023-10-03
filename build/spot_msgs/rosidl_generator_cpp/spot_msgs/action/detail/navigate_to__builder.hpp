// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:action/NavigateTo.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__ACTION__DETAIL__NAVIGATE_TO__BUILDER_HPP_
#define SPOT_MSGS__ACTION__DETAIL__NAVIGATE_TO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/action/detail/navigate_to__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateTo_Goal_initial_localization_waypoint
{
public:
  explicit Init_NavigateTo_Goal_initial_localization_waypoint(::spot_msgs::action::NavigateTo_Goal & msg)
  : msg_(msg)
  {}
  ::spot_msgs::action::NavigateTo_Goal initial_localization_waypoint(::spot_msgs::action::NavigateTo_Goal::_initial_localization_waypoint_type arg)
  {
    msg_.initial_localization_waypoint = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::NavigateTo_Goal msg_;
};

class Init_NavigateTo_Goal_initial_localization_fiducial
{
public:
  explicit Init_NavigateTo_Goal_initial_localization_fiducial(::spot_msgs::action::NavigateTo_Goal & msg)
  : msg_(msg)
  {}
  Init_NavigateTo_Goal_initial_localization_waypoint initial_localization_fiducial(::spot_msgs::action::NavigateTo_Goal::_initial_localization_fiducial_type arg)
  {
    msg_.initial_localization_fiducial = std::move(arg);
    return Init_NavigateTo_Goal_initial_localization_waypoint(msg_);
  }

private:
  ::spot_msgs::action::NavigateTo_Goal msg_;
};

class Init_NavigateTo_Goal_navigate_to
{
public:
  explicit Init_NavigateTo_Goal_navigate_to(::spot_msgs::action::NavigateTo_Goal & msg)
  : msg_(msg)
  {}
  Init_NavigateTo_Goal_initial_localization_fiducial navigate_to(::spot_msgs::action::NavigateTo_Goal::_navigate_to_type arg)
  {
    msg_.navigate_to = std::move(arg);
    return Init_NavigateTo_Goal_initial_localization_fiducial(msg_);
  }

private:
  ::spot_msgs::action::NavigateTo_Goal msg_;
};

class Init_NavigateTo_Goal_upload_path
{
public:
  Init_NavigateTo_Goal_upload_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateTo_Goal_navigate_to upload_path(::spot_msgs::action::NavigateTo_Goal::_upload_path_type arg)
  {
    msg_.upload_path = std::move(arg);
    return Init_NavigateTo_Goal_navigate_to(msg_);
  }

private:
  ::spot_msgs::action::NavigateTo_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::NavigateTo_Goal>()
{
  return spot_msgs::action::builder::Init_NavigateTo_Goal_upload_path();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateTo_Result_message
{
public:
  explicit Init_NavigateTo_Result_message(::spot_msgs::action::NavigateTo_Result & msg)
  : msg_(msg)
  {}
  ::spot_msgs::action::NavigateTo_Result message(::spot_msgs::action::NavigateTo_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::NavigateTo_Result msg_;
};

class Init_NavigateTo_Result_success
{
public:
  Init_NavigateTo_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateTo_Result_message success(::spot_msgs::action::NavigateTo_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_NavigateTo_Result_message(msg_);
  }

private:
  ::spot_msgs::action::NavigateTo_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::NavigateTo_Result>()
{
  return spot_msgs::action::builder::Init_NavigateTo_Result_success();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateTo_Feedback_waypoint_id
{
public:
  Init_NavigateTo_Feedback_waypoint_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::action::NavigateTo_Feedback waypoint_id(::spot_msgs::action::NavigateTo_Feedback::_waypoint_id_type arg)
  {
    msg_.waypoint_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::NavigateTo_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::NavigateTo_Feedback>()
{
  return spot_msgs::action::builder::Init_NavigateTo_Feedback_waypoint_id();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateTo_SendGoal_Request_goal
{
public:
  explicit Init_NavigateTo_SendGoal_Request_goal(::spot_msgs::action::NavigateTo_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::spot_msgs::action::NavigateTo_SendGoal_Request goal(::spot_msgs::action::NavigateTo_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::NavigateTo_SendGoal_Request msg_;
};

class Init_NavigateTo_SendGoal_Request_goal_id
{
public:
  Init_NavigateTo_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateTo_SendGoal_Request_goal goal_id(::spot_msgs::action::NavigateTo_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_NavigateTo_SendGoal_Request_goal(msg_);
  }

private:
  ::spot_msgs::action::NavigateTo_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::NavigateTo_SendGoal_Request>()
{
  return spot_msgs::action::builder::Init_NavigateTo_SendGoal_Request_goal_id();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateTo_SendGoal_Response_stamp
{
public:
  explicit Init_NavigateTo_SendGoal_Response_stamp(::spot_msgs::action::NavigateTo_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::spot_msgs::action::NavigateTo_SendGoal_Response stamp(::spot_msgs::action::NavigateTo_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::NavigateTo_SendGoal_Response msg_;
};

class Init_NavigateTo_SendGoal_Response_accepted
{
public:
  Init_NavigateTo_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateTo_SendGoal_Response_stamp accepted(::spot_msgs::action::NavigateTo_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_NavigateTo_SendGoal_Response_stamp(msg_);
  }

private:
  ::spot_msgs::action::NavigateTo_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::NavigateTo_SendGoal_Response>()
{
  return spot_msgs::action::builder::Init_NavigateTo_SendGoal_Response_accepted();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateTo_GetResult_Request_goal_id
{
public:
  Init_NavigateTo_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::action::NavigateTo_GetResult_Request goal_id(::spot_msgs::action::NavigateTo_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::NavigateTo_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::NavigateTo_GetResult_Request>()
{
  return spot_msgs::action::builder::Init_NavigateTo_GetResult_Request_goal_id();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateTo_GetResult_Response_result
{
public:
  explicit Init_NavigateTo_GetResult_Response_result(::spot_msgs::action::NavigateTo_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::spot_msgs::action::NavigateTo_GetResult_Response result(::spot_msgs::action::NavigateTo_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::NavigateTo_GetResult_Response msg_;
};

class Init_NavigateTo_GetResult_Response_status
{
public:
  Init_NavigateTo_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateTo_GetResult_Response_result status(::spot_msgs::action::NavigateTo_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_NavigateTo_GetResult_Response_result(msg_);
  }

private:
  ::spot_msgs::action::NavigateTo_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::NavigateTo_GetResult_Response>()
{
  return spot_msgs::action::builder::Init_NavigateTo_GetResult_Response_status();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace action
{

namespace builder
{

class Init_NavigateTo_FeedbackMessage_feedback
{
public:
  explicit Init_NavigateTo_FeedbackMessage_feedback(::spot_msgs::action::NavigateTo_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::spot_msgs::action::NavigateTo_FeedbackMessage feedback(::spot_msgs::action::NavigateTo_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::action::NavigateTo_FeedbackMessage msg_;
};

class Init_NavigateTo_FeedbackMessage_goal_id
{
public:
  Init_NavigateTo_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateTo_FeedbackMessage_feedback goal_id(::spot_msgs::action::NavigateTo_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_NavigateTo_FeedbackMessage_feedback(msg_);
  }

private:
  ::spot_msgs::action::NavigateTo_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::action::NavigateTo_FeedbackMessage>()
{
  return spot_msgs::action::builder::Init_NavigateTo_FeedbackMessage_goal_id();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__ACTION__DETAIL__NAVIGATE_TO__BUILDER_HPP_
