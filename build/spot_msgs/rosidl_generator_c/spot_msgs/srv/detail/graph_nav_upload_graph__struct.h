// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:srv/GraphNavUploadGraph.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_UPLOAD_GRAPH__STRUCT_H_
#define SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_UPLOAD_GRAPH__STRUCT_H_

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

/// Struct defined in srv/GraphNavUploadGraph in the package spot_msgs.
typedef struct spot_msgs__srv__GraphNavUploadGraph_Request
{
  rosidl_runtime_c__String upload_filepath;
} spot_msgs__srv__GraphNavUploadGraph_Request;

// Struct for a sequence of spot_msgs__srv__GraphNavUploadGraph_Request.
typedef struct spot_msgs__srv__GraphNavUploadGraph_Request__Sequence
{
  spot_msgs__srv__GraphNavUploadGraph_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__srv__GraphNavUploadGraph_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GraphNavUploadGraph in the package spot_msgs.
typedef struct spot_msgs__srv__GraphNavUploadGraph_Response
{
  bool success;
  rosidl_runtime_c__String message;
} spot_msgs__srv__GraphNavUploadGraph_Response;

// Struct for a sequence of spot_msgs__srv__GraphNavUploadGraph_Response.
typedef struct spot_msgs__srv__GraphNavUploadGraph_Response__Sequence
{
  spot_msgs__srv__GraphNavUploadGraph_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__srv__GraphNavUploadGraph_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_UPLOAD_GRAPH__STRUCT_H_
