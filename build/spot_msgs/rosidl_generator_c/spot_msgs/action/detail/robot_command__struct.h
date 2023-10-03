// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:action/RobotCommand.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__ACTION__DETAIL__ROBOT_COMMAND__STRUCT_H_
#define SPOT_MSGS__ACTION__DETAIL__ROBOT_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "bosdyn_msgs/msg/detail/robot_command__struct.h"

/// Struct defined in action/RobotCommand in the package spot_msgs.
typedef struct spot_msgs__action__RobotCommand_Goal
{
  bosdyn_msgs__msg__RobotCommand command;
} spot_msgs__action__RobotCommand_Goal;

// Struct for a sequence of spot_msgs__action__RobotCommand_Goal.
typedef struct spot_msgs__action__RobotCommand_Goal__Sequence
{
  spot_msgs__action__RobotCommand_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__RobotCommand_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
#include "bosdyn_msgs/msg/detail/robot_command_feedback__struct.h"
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/RobotCommand in the package spot_msgs.
typedef struct spot_msgs__action__RobotCommand_Result
{
  bosdyn_msgs__msg__RobotCommandFeedback result;
  bool success;
  rosidl_runtime_c__String message;
} spot_msgs__action__RobotCommand_Result;

// Struct for a sequence of spot_msgs__action__RobotCommand_Result.
typedef struct spot_msgs__action__RobotCommand_Result__Sequence
{
  spot_msgs__action__RobotCommand_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__RobotCommand_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'feedback'
// already included above
// #include "bosdyn_msgs/msg/detail/robot_command_feedback__struct.h"

/// Struct defined in action/RobotCommand in the package spot_msgs.
typedef struct spot_msgs__action__RobotCommand_Feedback
{
  bosdyn_msgs__msg__RobotCommandFeedback feedback;
} spot_msgs__action__RobotCommand_Feedback;

// Struct for a sequence of spot_msgs__action__RobotCommand_Feedback.
typedef struct spot_msgs__action__RobotCommand_Feedback__Sequence
{
  spot_msgs__action__RobotCommand_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__RobotCommand_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "spot_msgs/action/detail/robot_command__struct.h"

/// Struct defined in action/RobotCommand in the package spot_msgs.
typedef struct spot_msgs__action__RobotCommand_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  spot_msgs__action__RobotCommand_Goal goal;
} spot_msgs__action__RobotCommand_SendGoal_Request;

// Struct for a sequence of spot_msgs__action__RobotCommand_SendGoal_Request.
typedef struct spot_msgs__action__RobotCommand_SendGoal_Request__Sequence
{
  spot_msgs__action__RobotCommand_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__RobotCommand_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/RobotCommand in the package spot_msgs.
typedef struct spot_msgs__action__RobotCommand_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} spot_msgs__action__RobotCommand_SendGoal_Response;

// Struct for a sequence of spot_msgs__action__RobotCommand_SendGoal_Response.
typedef struct spot_msgs__action__RobotCommand_SendGoal_Response__Sequence
{
  spot_msgs__action__RobotCommand_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__RobotCommand_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/RobotCommand in the package spot_msgs.
typedef struct spot_msgs__action__RobotCommand_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} spot_msgs__action__RobotCommand_GetResult_Request;

// Struct for a sequence of spot_msgs__action__RobotCommand_GetResult_Request.
typedef struct spot_msgs__action__RobotCommand_GetResult_Request__Sequence
{
  spot_msgs__action__RobotCommand_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__RobotCommand_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.h"

/// Struct defined in action/RobotCommand in the package spot_msgs.
typedef struct spot_msgs__action__RobotCommand_GetResult_Response
{
  int8_t status;
  spot_msgs__action__RobotCommand_Result result;
} spot_msgs__action__RobotCommand_GetResult_Response;

// Struct for a sequence of spot_msgs__action__RobotCommand_GetResult_Response.
typedef struct spot_msgs__action__RobotCommand_GetResult_Response__Sequence
{
  spot_msgs__action__RobotCommand_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__RobotCommand_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.h"

/// Struct defined in action/RobotCommand in the package spot_msgs.
typedef struct spot_msgs__action__RobotCommand_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  spot_msgs__action__RobotCommand_Feedback feedback;
} spot_msgs__action__RobotCommand_FeedbackMessage;

// Struct for a sequence of spot_msgs__action__RobotCommand_FeedbackMessage.
typedef struct spot_msgs__action__RobotCommand_FeedbackMessage__Sequence
{
  spot_msgs__action__RobotCommand_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__RobotCommand_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__ACTION__DETAIL__ROBOT_COMMAND__STRUCT_H_
