// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from spot_msgs:msg/LeaseResource.idl
// generated code does not contain a copyright notice
#include "spot_msgs/msg/detail/lease_resource__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "spot_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "spot_msgs/msg/detail/lease_resource__struct.h"
#include "spot_msgs/msg/detail/lease_resource__functions.h"
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

#include "rosidl_runtime_c/string.h"  // resource
#include "rosidl_runtime_c/string_functions.h"  // resource
#include "spot_msgs/msg/detail/lease__functions.h"  // lease
#include "spot_msgs/msg/detail/lease_owner__functions.h"  // lease_owner

// forward declare type support functions
size_t get_serialized_size_spot_msgs__msg__Lease(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_spot_msgs__msg__Lease(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, msg, Lease)();
size_t get_serialized_size_spot_msgs__msg__LeaseOwner(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_spot_msgs__msg__LeaseOwner(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, msg, LeaseOwner)();


using _LeaseResource__ros_msg_type = spot_msgs__msg__LeaseResource;

static bool _LeaseResource__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _LeaseResource__ros_msg_type * ros_message = static_cast<const _LeaseResource__ros_msg_type *>(untyped_ros_message);
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

  // Field name: lease
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, spot_msgs, msg, Lease
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->lease, cdr))
    {
      return false;
    }
  }

  // Field name: lease_owner
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, spot_msgs, msg, LeaseOwner
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->lease_owner, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _LeaseResource__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _LeaseResource__ros_msg_type * ros_message = static_cast<_LeaseResource__ros_msg_type *>(untyped_ros_message);
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

  // Field name: lease
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, spot_msgs, msg, Lease
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->lease))
    {
      return false;
    }
  }

  // Field name: lease_owner
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, spot_msgs, msg, LeaseOwner
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->lease_owner))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_spot_msgs
size_t get_serialized_size_spot_msgs__msg__LeaseResource(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LeaseResource__ros_msg_type * ros_message = static_cast<const _LeaseResource__ros_msg_type *>(untyped_ros_message);
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
  // field.name lease

  current_alignment += get_serialized_size_spot_msgs__msg__Lease(
    &(ros_message->lease), current_alignment);
  // field.name lease_owner

  current_alignment += get_serialized_size_spot_msgs__msg__LeaseOwner(
    &(ros_message->lease_owner), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _LeaseResource__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_spot_msgs__msg__LeaseResource(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_spot_msgs
size_t max_serialized_size_spot_msgs__msg__LeaseResource(
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
  // member: lease
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      current_alignment +=
        max_serialized_size_spot_msgs__msg__Lease(
        inner_full_bounded, inner_is_plain, current_alignment);
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: lease_owner
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      current_alignment +=
        max_serialized_size_spot_msgs__msg__LeaseOwner(
        inner_full_bounded, inner_is_plain, current_alignment);
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _LeaseResource__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_spot_msgs__msg__LeaseResource(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_LeaseResource = {
  "spot_msgs::msg",
  "LeaseResource",
  _LeaseResource__cdr_serialize,
  _LeaseResource__cdr_deserialize,
  _LeaseResource__get_serialized_size,
  _LeaseResource__max_serialized_size
};

static rosidl_message_type_support_t _LeaseResource__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_LeaseResource,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, msg, LeaseResource)() {
  return &_LeaseResource__type_support;
}

#if defined(__cplusplus)
}
#endif
