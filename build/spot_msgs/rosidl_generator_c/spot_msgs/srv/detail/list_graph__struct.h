// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:srv/ListGraph.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__LIST_GRAPH__STRUCT_H_
#define SPOT_MSGS__SRV__DETAIL__LIST_GRAPH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'upload_filepath'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ListGraph in the package spot_msgs.
typedef struct spot_msgs__srv__ListGraph_Request
{
  rosidl_runtime_c__String upload_filepath;
} spot_msgs__srv__ListGraph_Request;

// Struct for a sequence of spot_msgs__srv__ListGraph_Request.
typedef struct spot_msgs__srv__ListGraph_Request__Sequence
{
  spot_msgs__srv__ListGraph_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__srv__ListGraph_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'waypoint_ids'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ListGraph in the package spot_msgs.
typedef struct spot_msgs__srv__ListGraph_Response
{
  rosidl_runtime_c__String__Sequence waypoint_ids;
} spot_msgs__srv__ListGraph_Response;

// Struct for a sequence of spot_msgs__srv__ListGraph_Response.
typedef struct spot_msgs__srv__ListGraph_Response__Sequence
{
  spot_msgs__srv__ListGraph_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__srv__ListGraph_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__SRV__DETAIL__LIST_GRAPH__STRUCT_H_
