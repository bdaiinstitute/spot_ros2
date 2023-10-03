// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from spot_msgs:action/RobotCommand.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__ACTION__DETAIL__ROBOT_COMMAND__STRUCT_HPP_
#define SPOT_MSGS__ACTION__DETAIL__ROBOT_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'command'
#include "bosdyn_msgs/msg/detail/robot_command__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__spot_msgs__action__RobotCommand_Goal __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__action__RobotCommand_Goal __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RobotCommand_Goal_
{
  using Type = RobotCommand_Goal_<ContainerAllocator>;

  explicit RobotCommand_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : command(_init)
  {
    (void)_init;
  }

  explicit RobotCommand_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : command(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _command_type =
    bosdyn_msgs::msg::RobotCommand_<ContainerAllocator>;
  _command_type command;

  // setters for named parameter idiom
  Type & set__command(
    const bosdyn_msgs::msg::RobotCommand_<ContainerAllocator> & _arg)
  {
    this->command = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    spot_msgs::action::RobotCommand_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::action::RobotCommand_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::action::RobotCommand_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::action::RobotCommand_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::action::RobotCommand_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::action::RobotCommand_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::action::RobotCommand_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::action::RobotCommand_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::action::RobotCommand_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::action::RobotCommand_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__action__RobotCommand_Goal
    std::shared_ptr<spot_msgs::action::RobotCommand_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__action__RobotCommand_Goal
    std::shared_ptr<spot_msgs::action::RobotCommand_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotCommand_Goal_ & other) const
  {
    if (this->command != other.command) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotCommand_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotCommand_Goal_

// alias to use template instance with default allocator
using RobotCommand_Goal =
  spot_msgs::action::RobotCommand_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace spot_msgs


// Include directives for member types
// Member 'result'
#include "bosdyn_msgs/msg/detail/robot_command_feedback__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__spot_msgs__action__RobotCommand_Result __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__action__RobotCommand_Result __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RobotCommand_Result_
{
  using Type = RobotCommand_Result_<ContainerAllocator>;

  explicit RobotCommand_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit RobotCommand_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init),
    message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _result_type =
    bosdyn_msgs::msg::RobotCommandFeedback_<ContainerAllocator>;
  _result_type result;
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__result(
    const bosdyn_msgs::msg::RobotCommandFeedback_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    spot_msgs::action::RobotCommand_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::action::RobotCommand_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::action::RobotCommand_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::action::RobotCommand_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::action::RobotCommand_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::action::RobotCommand_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::action::RobotCommand_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::action::RobotCommand_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::action::RobotCommand_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::action::RobotCommand_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__action__RobotCommand_Result
    std::shared_ptr<spot_msgs::action::RobotCommand_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__action__RobotCommand_Result
    std::shared_ptr<spot_msgs::action::RobotCommand_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotCommand_Result_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotCommand_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotCommand_Result_

// alias to use template instance with default allocator
using RobotCommand_Result =
  spot_msgs::action::RobotCommand_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace spot_msgs


// Include directives for member types
// Member 'feedback'
// already included above
// #include "bosdyn_msgs/msg/detail/robot_command_feedback__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__spot_msgs__action__RobotCommand_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__action__RobotCommand_Feedback __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RobotCommand_Feedback_
{
  using Type = RobotCommand_Feedback_<ContainerAllocator>;

  explicit RobotCommand_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : feedback(_init)
  {
    (void)_init;
  }

  explicit RobotCommand_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _feedback_type =
    bosdyn_msgs::msg::RobotCommandFeedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__feedback(
    const bosdyn_msgs::msg::RobotCommandFeedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    spot_msgs::action::RobotCommand_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::action::RobotCommand_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::action::RobotCommand_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::action::RobotCommand_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::action::RobotCommand_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::action::RobotCommand_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::action::RobotCommand_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::action::RobotCommand_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::action::RobotCommand_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::action::RobotCommand_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__action__RobotCommand_Feedback
    std::shared_ptr<spot_msgs::action::RobotCommand_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__action__RobotCommand_Feedback
    std::shared_ptr<spot_msgs::action::RobotCommand_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotCommand_Feedback_ & other) const
  {
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotCommand_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotCommand_Feedback_

// alias to use template instance with default allocator
using RobotCommand_Feedback =
  spot_msgs::action::RobotCommand_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace spot_msgs


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "spot_msgs/action/detail/robot_command__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__spot_msgs__action__RobotCommand_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__action__RobotCommand_SendGoal_Request __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RobotCommand_SendGoal_Request_
{
  using Type = RobotCommand_SendGoal_Request_<ContainerAllocator>;

  explicit RobotCommand_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit RobotCommand_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    spot_msgs::action::RobotCommand_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const spot_msgs::action::RobotCommand_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    spot_msgs::action::RobotCommand_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::action::RobotCommand_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::action::RobotCommand_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::action::RobotCommand_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::action::RobotCommand_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::action::RobotCommand_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::action::RobotCommand_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::action::RobotCommand_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::action::RobotCommand_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::action::RobotCommand_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__action__RobotCommand_SendGoal_Request
    std::shared_ptr<spot_msgs::action::RobotCommand_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__action__RobotCommand_SendGoal_Request
    std::shared_ptr<spot_msgs::action::RobotCommand_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotCommand_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotCommand_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotCommand_SendGoal_Request_

// alias to use template instance with default allocator
using RobotCommand_SendGoal_Request =
  spot_msgs::action::RobotCommand_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace spot_msgs


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__spot_msgs__action__RobotCommand_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__action__RobotCommand_SendGoal_Response __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RobotCommand_SendGoal_Response_
{
  using Type = RobotCommand_SendGoal_Response_<ContainerAllocator>;

  explicit RobotCommand_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit RobotCommand_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    spot_msgs::action::RobotCommand_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::action::RobotCommand_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::action::RobotCommand_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::action::RobotCommand_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::action::RobotCommand_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::action::RobotCommand_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::action::RobotCommand_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::action::RobotCommand_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::action::RobotCommand_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::action::RobotCommand_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__action__RobotCommand_SendGoal_Response
    std::shared_ptr<spot_msgs::action::RobotCommand_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__action__RobotCommand_SendGoal_Response
    std::shared_ptr<spot_msgs::action::RobotCommand_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotCommand_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotCommand_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotCommand_SendGoal_Response_

// alias to use template instance with default allocator
using RobotCommand_SendGoal_Response =
  spot_msgs::action::RobotCommand_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace spot_msgs

namespace spot_msgs
{

namespace action
{

struct RobotCommand_SendGoal
{
  using Request = spot_msgs::action::RobotCommand_SendGoal_Request;
  using Response = spot_msgs::action::RobotCommand_SendGoal_Response;
};

}  // namespace action

}  // namespace spot_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__spot_msgs__action__RobotCommand_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__action__RobotCommand_GetResult_Request __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RobotCommand_GetResult_Request_
{
  using Type = RobotCommand_GetResult_Request_<ContainerAllocator>;

  explicit RobotCommand_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit RobotCommand_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    spot_msgs::action::RobotCommand_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::action::RobotCommand_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::action::RobotCommand_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::action::RobotCommand_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::action::RobotCommand_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::action::RobotCommand_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::action::RobotCommand_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::action::RobotCommand_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::action::RobotCommand_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::action::RobotCommand_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__action__RobotCommand_GetResult_Request
    std::shared_ptr<spot_msgs::action::RobotCommand_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__action__RobotCommand_GetResult_Request
    std::shared_ptr<spot_msgs::action::RobotCommand_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotCommand_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotCommand_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotCommand_GetResult_Request_

// alias to use template instance with default allocator
using RobotCommand_GetResult_Request =
  spot_msgs::action::RobotCommand_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace spot_msgs


// Include directives for member types
// Member 'result'
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__spot_msgs__action__RobotCommand_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__action__RobotCommand_GetResult_Response __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RobotCommand_GetResult_Response_
{
  using Type = RobotCommand_GetResult_Response_<ContainerAllocator>;

  explicit RobotCommand_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit RobotCommand_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    spot_msgs::action::RobotCommand_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const spot_msgs::action::RobotCommand_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    spot_msgs::action::RobotCommand_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::action::RobotCommand_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::action::RobotCommand_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::action::RobotCommand_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::action::RobotCommand_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::action::RobotCommand_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::action::RobotCommand_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::action::RobotCommand_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::action::RobotCommand_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::action::RobotCommand_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__action__RobotCommand_GetResult_Response
    std::shared_ptr<spot_msgs::action::RobotCommand_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__action__RobotCommand_GetResult_Response
    std::shared_ptr<spot_msgs::action::RobotCommand_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotCommand_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotCommand_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotCommand_GetResult_Response_

// alias to use template instance with default allocator
using RobotCommand_GetResult_Response =
  spot_msgs::action::RobotCommand_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace spot_msgs

namespace spot_msgs
{

namespace action
{

struct RobotCommand_GetResult
{
  using Request = spot_msgs::action::RobotCommand_GetResult_Request;
  using Response = spot_msgs::action::RobotCommand_GetResult_Response;
};

}  // namespace action

}  // namespace spot_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__spot_msgs__action__RobotCommand_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__action__RobotCommand_FeedbackMessage __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct RobotCommand_FeedbackMessage_
{
  using Type = RobotCommand_FeedbackMessage_<ContainerAllocator>;

  explicit RobotCommand_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit RobotCommand_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    spot_msgs::action::RobotCommand_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const spot_msgs::action::RobotCommand_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    spot_msgs::action::RobotCommand_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::action::RobotCommand_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::action::RobotCommand_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::action::RobotCommand_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::action::RobotCommand_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::action::RobotCommand_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::action::RobotCommand_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::action::RobotCommand_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::action::RobotCommand_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::action::RobotCommand_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__action__RobotCommand_FeedbackMessage
    std::shared_ptr<spot_msgs::action::RobotCommand_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__action__RobotCommand_FeedbackMessage
    std::shared_ptr<spot_msgs::action::RobotCommand_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotCommand_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotCommand_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotCommand_FeedbackMessage_

// alias to use template instance with default allocator
using RobotCommand_FeedbackMessage =
  spot_msgs::action::RobotCommand_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace spot_msgs

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace spot_msgs
{

namespace action
{

struct RobotCommand
{
  /// The goal message defined in the action definition.
  using Goal = spot_msgs::action::RobotCommand_Goal;
  /// The result message defined in the action definition.
  using Result = spot_msgs::action::RobotCommand_Result;
  /// The feedback message defined in the action definition.
  using Feedback = spot_msgs::action::RobotCommand_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = spot_msgs::action::RobotCommand_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = spot_msgs::action::RobotCommand_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = spot_msgs::action::RobotCommand_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct RobotCommand RobotCommand;

}  // namespace action

}  // namespace spot_msgs

#endif  // SPOT_MSGS__ACTION__DETAIL__ROBOT_COMMAND__STRUCT_HPP_
