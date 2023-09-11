// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from spot_msgs:msg/Lease.idl
// generated code does not contain a copyright notice
#include "spot_msgs/msg/detail/lease__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "spot_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "spot_msgs/msg/detail/lease__struct.h"
#include "spot_msgs/msg/detail/lease__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // sequence
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // sequence
#include "rosidl_runtime_c/string.h"  // epoch, resource
#include "rosidl_runtime_c/string_functions.h"  // epoch, resource

// forward declare type support functions


using _Lease__ros_msg_type = spot_msgs__msg__Lease;

static bool _Lease__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Lease__ros_msg_type * ros_message = static_cast<const _Lease__ros_msg_type *>(untyped_ros_message);
  // Field name: resource
  {
    const rosidl_runtime_c__String * str = &ros_message->resource;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: epoch
  {
    const rosidl_runtime_c__String * str = &ros_message->epoch;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: sequence
  {
    size_t size = ros_message->sequence.size;
    auto array_ptr = ros_message->sequence.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _Lease__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Lease__ros_msg_type * ros_message = static_cast<_Lease__ros_msg_type *>(untyped_ros_message);
  // Field name: resource
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->resource.data) {
      rosidl_runtime_c__String__init(&ros_message->resource);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->resource,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'resource'\n");
      return false;
    }
  }

  // Field name: epoch
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->epoch.data) {
      rosidl_runtime_c__String__init(&ros_message->epoch);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->epoch,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'epoch'\n");
      return false;
    }
  }

  // Field name: sequence
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->sequence.data) {
      rosidl_runtime_c__uint32__Sequence__fini(&ros_message->sequence);
    }
    if (!rosidl_runtime_c__uint32__Sequence__init(&ros_message->sequence, size)) {
      fprintf(stderr, "failed to create array for field 'sequence'");
      return false;
    }
    auto array_ptr = ros_message->sequence.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_spot_msgs
size_t get_serialized_size_spot_msgs__msg__Lease(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Lease__ros_msg_type * ros_message = static_cast<const _Lease__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name resource
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->resource.size + 1);
  // field.name epoch
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->epoch.size + 1);
  // field.name sequence
  {
    size_t array_size = ros_message->sequence.size;
    auto array_ptr = ros_message->sequence.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Lease__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_spot_msgs__msg__Lease(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_spot_msgs
size_t max_serialized_size_spot_msgs__msg__Lease(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: resource
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: epoch
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: sequence
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _Lease__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_spot_msgs__msg__Lease(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Lease = {
  "spot_msgs::msg",
  "Lease",
  _Lease__cdr_serialize,
  _Lease__cdr_deserialize,
  _Lease__get_serialized_size,
  _Lease__max_serialized_size
};

static rosidl_message_type_support_t _Lease__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Lease,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, msg, Lease)() {
  return &_Lease__type_support;
}

#if defined(__cplusplus)
}
#endif
