// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:action/NavigateTo.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__ACTION__DETAIL__NAVIGATE_TO__STRUCT_H_
#define SPOT_MSGS__ACTION__DETAIL__NAVIGATE_TO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'upload_path'
// Member 'navigate_to'
// Member 'initial_localization_waypoint'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/NavigateTo in the package spot_msgs.
typedef struct spot_msgs__action__NavigateTo_Goal
{
  /// Absolute path to map_directory, which is downloaded from tablet controller
  rosidl_runtime_c__String upload_path;
  /// Waypoint id string for where to go
  rosidl_runtime_c__String navigate_to;
  /// Tells the initializer whether to use fiducials
  bool initial_localization_fiducial;
  /// Waypoint id to trigger localization
  rosidl_runtime_c__String initial_localization_waypoint;
} spot_msgs__action__NavigateTo_Goal;

// Struct for a sequence of spot_msgs__action__NavigateTo_Goal.
typedef struct spot_msgs__action__NavigateTo_Goal__Sequence
{
  spot_msgs__action__NavigateTo_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__NavigateTo_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/NavigateTo in the package spot_msgs.
typedef struct spot_msgs__action__NavigateTo_Result
{
  /// indicate successful run of triggered service
  bool success;
  /// informational, e.g. for error messages
  rosidl_runtime_c__String message;
} spot_msgs__action__NavigateTo_Result;

// Struct for a sequence of spot_msgs__action__NavigateTo_Result.
typedef struct spot_msgs__action__NavigateTo_Result__Sequence
{
  spot_msgs__action__NavigateTo_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__NavigateTo_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'waypoint_id'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/NavigateTo in the package spot_msgs.
typedef struct spot_msgs__action__NavigateTo_Feedback
{
  rosidl_runtime_c__String waypoint_id;
} spot_msgs__action__NavigateTo_Feedback;

// Struct for a sequence of spot_msgs__action__NavigateTo_Feedback.
typedef struct spot_msgs__action__NavigateTo_Feedback__Sequence
{
  spot_msgs__action__NavigateTo_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__NavigateTo_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "spot_msgs/action/detail/navigate_to__struct.h"

/// Struct defined in action/NavigateTo in the package spot_msgs.
typedef struct spot_msgs__action__NavigateTo_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  spot_msgs__action__NavigateTo_Goal goal;
} spot_msgs__action__NavigateTo_SendGoal_Request;

// Struct for a sequence of spot_msgs__action__NavigateTo_SendGoal_Request.
typedef struct spot_msgs__action__NavigateTo_SendGoal_Request__Sequence
{
  spot_msgs__action__NavigateTo_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__NavigateTo_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/NavigateTo in the package spot_msgs.
typedef struct spot_msgs__action__NavigateTo_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} spot_msgs__action__NavigateTo_SendGoal_Response;

// Struct for a sequence of spot_msgs__action__NavigateTo_SendGoal_Response.
typedef struct spot_msgs__action__NavigateTo_SendGoal_Response__Sequence
{
  spot_msgs__action__NavigateTo_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__NavigateTo_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/NavigateTo in the package spot_msgs.
typedef struct spot_msgs__action__NavigateTo_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} spot_msgs__action__NavigateTo_GetResult_Request;

// Struct for a sequence of spot_msgs__action__NavigateTo_GetResult_Request.
typedef struct spot_msgs__action__NavigateTo_GetResult_Request__Sequence
{
  spot_msgs__action__NavigateTo_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__NavigateTo_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "spot_msgs/action/detail/navigate_to__struct.h"

/// Struct defined in action/NavigateTo in the package spot_msgs.
typedef struct spot_msgs__action__NavigateTo_GetResult_Response
{
  int8_t status;
  spot_msgs__action__NavigateTo_Result result;
} spot_msgs__action__NavigateTo_GetResult_Response;

// Struct for a sequence of spot_msgs__action__NavigateTo_GetResult_Response.
typedef struct spot_msgs__action__NavigateTo_GetResult_Response__Sequence
{
  spot_msgs__action__NavigateTo_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__NavigateTo_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "spot_msgs/action/detail/navigate_to__struct.h"

/// Struct defined in action/NavigateTo in the package spot_msgs.
typedef struct spot_msgs__action__NavigateTo_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  spot_msgs__action__NavigateTo_Feedback feedback;
} spot_msgs__action__NavigateTo_FeedbackMessage;

// Struct for a sequence of spot_msgs__action__NavigateTo_FeedbackMessage.
typedef struct spot_msgs__action__NavigateTo_FeedbackMessage__Sequence
{
  spot_msgs__action__NavigateTo_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__NavigateTo_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__ACTION__DETAIL__NAVIGATE_TO__STRUCT_H_
