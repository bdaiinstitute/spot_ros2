// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:action/Trajectory.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__ACTION__DETAIL__TRAJECTORY__STRUCT_H_
#define SPOT_MSGS__ACTION__DETAIL__TRAJECTORY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'duration'
#include "builtin_interfaces/msg/detail/duration__struct.h"

/// Struct defined in action/Trajectory in the package spot_msgs.
typedef struct spot_msgs__action__Trajectory_Goal
{
  geometry_msgs__msg__PoseStamped target_pose;
  /// After this duration, the command will time out and the robot will stop. Must be non-zero
  builtin_interfaces__msg__Duration duration;
  /// If true, the feedback from the trajectory command must indicate that the robot is
  /// at the goal position. If set to false, the robot being near the goal is equivalent to
  /// it being at the goal. This is based on the feedback received from the boston dynamics
  /// API call at
  /// https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference.html?highlight=status_near_goal#se2trajectorycommand-feedback-status
  bool precise_positioning;
} spot_msgs__action__Trajectory_Goal;

// Struct for a sequence of spot_msgs__action__Trajectory_Goal.
typedef struct spot_msgs__action__Trajectory_Goal__Sequence
{
  spot_msgs__action__Trajectory_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__Trajectory_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/Trajectory in the package spot_msgs.
typedef struct spot_msgs__action__Trajectory_Result
{
  bool success;
  rosidl_runtime_c__String message;
} spot_msgs__action__Trajectory_Result;

// Struct for a sequence of spot_msgs__action__Trajectory_Result.
typedef struct spot_msgs__action__Trajectory_Result__Sequence
{
  spot_msgs__action__Trajectory_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__Trajectory_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'feedback'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/Trajectory in the package spot_msgs.
typedef struct spot_msgs__action__Trajectory_Feedback
{
  rosidl_runtime_c__String feedback;
} spot_msgs__action__Trajectory_Feedback;

// Struct for a sequence of spot_msgs__action__Trajectory_Feedback.
typedef struct spot_msgs__action__Trajectory_Feedback__Sequence
{
  spot_msgs__action__Trajectory_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__Trajectory_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "spot_msgs/action/detail/trajectory__struct.h"

/// Struct defined in action/Trajectory in the package spot_msgs.
typedef struct spot_msgs__action__Trajectory_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  spot_msgs__action__Trajectory_Goal goal;
} spot_msgs__action__Trajectory_SendGoal_Request;

// Struct for a sequence of spot_msgs__action__Trajectory_SendGoal_Request.
typedef struct spot_msgs__action__Trajectory_SendGoal_Request__Sequence
{
  spot_msgs__action__Trajectory_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__Trajectory_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Trajectory in the package spot_msgs.
typedef struct spot_msgs__action__Trajectory_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} spot_msgs__action__Trajectory_SendGoal_Response;

// Struct for a sequence of spot_msgs__action__Trajectory_SendGoal_Response.
typedef struct spot_msgs__action__Trajectory_SendGoal_Response__Sequence
{
  spot_msgs__action__Trajectory_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__Trajectory_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Trajectory in the package spot_msgs.
typedef struct spot_msgs__action__Trajectory_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} spot_msgs__action__Trajectory_GetResult_Request;

// Struct for a sequence of spot_msgs__action__Trajectory_GetResult_Request.
typedef struct spot_msgs__action__Trajectory_GetResult_Request__Sequence
{
  spot_msgs__action__Trajectory_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__Trajectory_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "spot_msgs/action/detail/trajectory__struct.h"

/// Struct defined in action/Trajectory in the package spot_msgs.
typedef struct spot_msgs__action__Trajectory_GetResult_Response
{
  int8_t status;
  spot_msgs__action__Trajectory_Result result;
} spot_msgs__action__Trajectory_GetResult_Response;

// Struct for a sequence of spot_msgs__action__Trajectory_GetResult_Response.
typedef struct spot_msgs__action__Trajectory_GetResult_Response__Sequence
{
  spot_msgs__action__Trajectory_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__Trajectory_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "spot_msgs/action/detail/trajectory__struct.h"

/// Struct defined in action/Trajectory in the package spot_msgs.
typedef struct spot_msgs__action__Trajectory_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  spot_msgs__action__Trajectory_Feedback feedback;
} spot_msgs__action__Trajectory_FeedbackMessage;

// Struct for a sequence of spot_msgs__action__Trajectory_FeedbackMessage.
typedef struct spot_msgs__action__Trajectory_FeedbackMessage__Sequence
{
  spot_msgs__action__Trajectory_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__action__Trajectory_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__ACTION__DETAIL__TRAJECTORY__STRUCT_H_
