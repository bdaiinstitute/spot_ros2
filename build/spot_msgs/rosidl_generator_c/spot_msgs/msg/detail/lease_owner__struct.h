// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:msg/LeaseOwner.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__LEASE_OWNER__STRUCT_H_
#define SPOT_MSGS__MSG__DETAIL__LEASE_OWNER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'client_name'
// Member 'user_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/LeaseOwner in the package spot_msgs.
typedef struct spot_msgs__msg__LeaseOwner
{
  rosidl_runtime_c__String client_name;
  rosidl_runtime_c__String user_name;
} spot_msgs__msg__LeaseOwner;

// Struct for a sequence of spot_msgs__msg__LeaseOwner.
typedef struct spot_msgs__msg__LeaseOwner__Sequence
{
  spot_msgs__msg__LeaseOwner * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__msg__LeaseOwner__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__MSG__DETAIL__LEASE_OWNER__STRUCT_H_
