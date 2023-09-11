// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:msg/Lease.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__LEASE__STRUCT_H_
#define SPOT_MSGS__MSG__DETAIL__LEASE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'resource'
// Member 'epoch'
#include "rosidl_runtime_c/string.h"
// Member 'sequence'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Lease in the package spot_msgs.
typedef struct spot_msgs__msg__Lease
{
  rosidl_runtime_c__String resource;
  rosidl_runtime_c__String epoch;
  rosidl_runtime_c__uint32__Sequence sequence;
} spot_msgs__msg__Lease;

// Struct for a sequence of spot_msgs__msg__Lease.
typedef struct spot_msgs__msg__Lease__Sequence
{
  spot_msgs__msg__Lease * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__msg__Lease__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__MSG__DETAIL__LEASE__STRUCT_H_
