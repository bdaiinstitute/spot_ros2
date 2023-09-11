// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:srv/ListWorldObjects.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__LIST_WORLD_OBJECTS__STRUCT_H_
#define SPOT_MSGS__SRV__DETAIL__LIST_WORLD_OBJECTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'request'
#include "bosdyn_msgs/msg/detail/list_world_object_request__struct.h"

/// Struct defined in srv/ListWorldObjects in the package spot_msgs.
typedef struct spot_msgs__srv__ListWorldObjects_Request
{
  bosdyn_msgs__msg__ListWorldObjectRequest request;
} spot_msgs__srv__ListWorldObjects_Request;

// Struct for a sequence of spot_msgs__srv__ListWorldObjects_Request.
typedef struct spot_msgs__srv__ListWorldObjects_Request__Sequence
{
  spot_msgs__srv__ListWorldObjects_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__srv__ListWorldObjects_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
#include "bosdyn_msgs/msg/detail/list_world_object_response__struct.h"

/// Struct defined in srv/ListWorldObjects in the package spot_msgs.
typedef struct spot_msgs__srv__ListWorldObjects_Response
{
  bosdyn_msgs__msg__ListWorldObjectResponse response;
} spot_msgs__srv__ListWorldObjects_Response;

// Struct for a sequence of spot_msgs__srv__ListWorldObjects_Response.
typedef struct spot_msgs__srv__ListWorldObjects_Response__Sequence
{
  spot_msgs__srv__ListWorldObjects_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__srv__ListWorldObjects_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__SRV__DETAIL__LIST_WORLD_OBJECTS__STRUCT_H_
