// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:action/Manipulation.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__ACTION__DETAIL__MANIPULATION__STRUCT_H_
#define SPOT_MSGS__ACTION__DETAIL__MANIPULATION__STRUCT_H_

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
#include "bosdyn_msgs/msg/detail/manipulation_api_request__struct.h"

/// Struct defined in action/Manipulation in the package spot_msgs.
typedef struct spot_msgs__action__Manipulation_Goal
{
  bosdyn_msgs__msg__ManipulationApiRequest command;
} spot_msgs__action__Manipulation_Goal;

// Struct for a sequence of spot_msgs__action__Manipulation_Goal.
typedef struct spot_msgs__action__Manipulation_Goal__Sequence
{
  spot_msgs__action__Manipulation_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__Manipulation_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
#include "bosdyn_msgs/msg/detail/manipulation_api_feedback_response__struct.h"
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/Manipulation in the package spot_msgs.
typedef struct spot_msgs__action__Manipulation_Result
{
  bosdyn_msgs__msg__ManipulationApiFeedbackResponse result;
  bool success;
  rosidl_runtime_c__String message;
} spot_msgs__action__Manipulation_Result;

// Struct for a sequence of spot_msgs__action__Manipulation_Result.
typedef struct spot_msgs__action__Manipulation_Result__Sequence
{
  spot_msgs__action__Manipulation_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__Manipulation_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'feedback'
// already included above
// #include "bosdyn_msgs/msg/detail/manipulation_api_feedback_response__struct.h"

/// Struct defined in action/Manipulation in the package spot_msgs.
typedef struct spot_msgs__action__Manipulation_Feedback
{
  bosdyn_msgs__msg__ManipulationApiFeedbackResponse feedback;
} spot_msgs__action__Manipulation_Feedback;

// Struct for a sequence of spot_msgs__action__Manipulation_Feedback.
typedef struct spot_msgs__action__Manipulation_Feedback__Sequence
{
  spot_msgs__action__Manipulation_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__Manipulation_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "spot_msgs/action/detail/manipulation__struct.h"

/// Struct defined in action/Manipulation in the package spot_msgs.
typedef struct spot_msgs__action__Manipulation_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  spot_msgs__action__Manipulation_Goal goal;
} spot_msgs__action__Manipulation_SendGoal_Request;

// Struct for a sequence of spot_msgs__action__Manipulation_SendGoal_Request.
typedef struct spot_msgs__action__Manipulation_SendGoal_Request__Sequence
{
  spot_msgs__action__Manipulation_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__Manipulation_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Manipulation in the package spot_msgs.
typedef struct spot_msgs__action__Manipulation_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} spot_msgs__action__Manipulation_SendGoal_Response;

// Struct for a sequence of spot_msgs__action__Manipulation_SendGoal_Response.
typedef struct spot_msgs__action__Manipulation_SendGoal_Response__Sequence
{
  spot_msgs__action__Manipulation_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__Manipulation_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Manipulation in the package spot_msgs.
typedef struct spot_msgs__action__Manipulation_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} spot_msgs__action__Manipulation_GetResult_Request;

// Struct for a sequence of spot_msgs__action__Manipulation_GetResult_Request.
typedef struct spot_msgs__action__Manipulation_GetResult_Request__Sequence
{
  spot_msgs__action__Manipulation_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__Manipulation_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "spot_msgs/action/detail/manipulation__struct.h"

/// Struct defined in action/Manipulation in the package spot_msgs.
typedef struct spot_msgs__action__Manipulation_GetResult_Response
{
  int8_t status;
  spot_msgs__action__Manipulation_Result result;
} spot_msgs__action__Manipulation_GetResult_Response;

// Struct for a sequence of spot_msgs__action__Manipulation_GetResult_Response.
typedef struct spot_msgs__action__Manipulation_GetResult_Response__Sequence
{
  spot_msgs__action__Manipulation_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__Manipulation_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "spot_msgs/action/detail/manipulation__struct.h"

/// Struct defined in action/Manipulation in the package spot_msgs.
typedef struct spot_msgs__action__Manipulation_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  spot_msgs__action__Manipulation_Feedback feedback;
} spot_msgs__action__Manipulation_FeedbackMessage;

// Struct for a sequence of spot_msgs__action__Manipulation_FeedbackMessage.
typedef struct spot_msgs__action__Manipulation_FeedbackMessage__Sequence
{
  spot_msgs__action__Manipulation_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__Manipulation_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__ACTION__DETAIL__MANIPULATION__STRUCT_H_
