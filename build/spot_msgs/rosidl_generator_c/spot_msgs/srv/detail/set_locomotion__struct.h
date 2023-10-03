// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:srv/SetLocomotion.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__SET_LOCOMOTION__STRUCT_H_
#define SPOT_MSGS__SRV__DETAIL__SET_LOCOMOTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetLocomotion in the package spot_msgs.
typedef struct spot_msgs__srv__SetLocomotion_Request
{
  /// See https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference.html?highlight=mobilityparams#locomotionhint for details
  uint32_t locomotion_mode;
} spot_msgs__srv__SetLocomotion_Request;

// Struct for a sequence of spot_msgs__srv__SetLocomotion_Request.
typedef struct spot_msgs__srv__SetLocomotion_Request__Sequence
{
  spot_msgs__srv__SetLocomotion_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__srv__SetLocomotion_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetLocomotion in the package spot_msgs.
typedef struct spot_msgs__srv__SetLocomotion_Response
{
  bool success;
  rosidl_runtime_c__String message;
} spot_msgs__srv__SetLocomotion_Response;

// Struct for a sequence of spot_msgs__srv__SetLocomotion_Response.
typedef struct spot_msgs__srv__SetLocomotion_Response__Sequence
{
  spot_msgs__srv__SetLocomotion_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__srv__SetLocomotion_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__SRV__DETAIL__SET_LOCOMOTION__STRUCT_H_
