// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:srv/GraphNavGetLocalizationPose.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_GET_LOCALIZATION_POSE__STRUCT_H_
#define SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_GET_LOCALIZATION_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GraphNavGetLocalizationPose in the package spot_msgs.
typedef struct spot_msgs__srv__GraphNavGetLocalizationPose_Request
{
  uint8_t structure_needs_at_least_one_member;
} spot_msgs__srv__GraphNavGetLocalizationPose_Request;

// Struct for a sequence of spot_msgs__srv__GraphNavGetLocalizationPose_Request.
typedef struct spot_msgs__srv__GraphNavGetLocalizationPose_Request__Sequence
{
  spot_msgs__srv__GraphNavGetLocalizationPose_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__srv__GraphNavGetLocalizationPose_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in srv/GraphNavGetLocalizationPose in the package spot_msgs.
typedef struct spot_msgs__srv__GraphNavGetLocalizationPose_Response
{
  bool success;
  rosidl_runtime_c__String message;
  geometry_msgs__msg__PoseStamped pose;
} spot_msgs__srv__GraphNavGetLocalizationPose_Response;

// Struct for a sequence of spot_msgs__srv__GraphNavGetLocalizationPose_Response.
typedef struct spot_msgs__srv__GraphNavGetLocalizationPose_Response__Sequence
{
  spot_msgs__srv__GraphNavGetLocalizationPose_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__srv__GraphNavGetLocalizationPose_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_GET_LOCALIZATION_POSE__STRUCT_H_
