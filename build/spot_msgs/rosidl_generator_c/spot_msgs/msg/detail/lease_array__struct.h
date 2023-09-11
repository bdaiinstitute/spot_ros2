// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:msg/LeaseArray.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__LEASE_ARRAY__STRUCT_H_
#define SPOT_MSGS__MSG__DETAIL__LEASE_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'resources'
#include "spot_msgs/msg/detail/lease_resource__struct.h"

/// Struct defined in msg/LeaseArray in the package spot_msgs.
typedef struct spot_msgs__msg__LeaseArray
{
  spot_msgs__msg__LeaseResource__Sequence resources;
} spot_msgs__msg__LeaseArray;

// Struct for a sequence of spot_msgs__msg__LeaseArray.
typedef struct spot_msgs__msg__LeaseArray__Sequence
{
  spot_msgs__msg__LeaseArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__msg__LeaseArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__MSG__DETAIL__LEASE_ARRAY__STRUCT_H_
