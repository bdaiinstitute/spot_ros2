// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from spot_msgs:msg/LeaseResource.idl
// generated code does not contain a copyright notice
#include "spot_msgs/msg/detail/lease_resource__rosidl_typesupport_fastrtps_cpp.hpp"
#include "spot_msgs/msg/detail/lease_resource__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace spot_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const spot_msgs::msg::Lease &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  spot_msgs::msg::Lease &);
size_t get_serialized_size(
  const spot_msgs::msg::Lease &,
  size_t current_alignment);
size_t
max_serialized_size_Lease(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace spot_msgs

namespace spot_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const spot_msgs::msg::LeaseOwner &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  spot_msgs::msg::LeaseOwner &);
size_t get_serialized_size(
  const spot_msgs::msg::LeaseOwner &,
  size_t current_alignment);
size_t
max_serialized_size_LeaseOwner(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace spot_msgs


namespace spot_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_spot_msgs
cdr_serialize(
  const spot_msgs::msg::LeaseResource & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: resource
  cdr << ros_message.resource;
  // Member: lease
  spot_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.lease,
    cdr);
  // Member: lease_owner
  spot_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.lease_owner,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_spot_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  spot_msgs::msg::LeaseResource & ros_message)
{
  // Member: resource
  cdr >> ros_message.resource;

  // Member: lease
  spot_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.lease);

  // Member: lease_owner
  spot_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.lease_owner);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_spot_msgs
get_serialized_size(
  const spot_msgs::msg::LeaseResource & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: resource
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.resource.size() + 1);
  // Member: lease

  current_alignment +=
    spot_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.lease, current_alignment);
  // Member: lease_owner

  current_alignment +=
    spot_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.lease_owner, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_spot_msgs
max_serialized_size_LeaseResource(
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


  // Member: resource
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

  // Member: lease
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      current_alignment +=
        spot_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Lease(
        inner_full_bounded, inner_is_plain, current_alignment);
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: lease_owner
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      current_alignment +=
        spot_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_LeaseOwner(
        inner_full_bounded, inner_is_plain, current_alignment);
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  return current_alignment - initial_alignment;
}

static bool _LeaseResource__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const spot_msgs::msg::LeaseResource *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _LeaseResource__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<spot_msgs::msg::LeaseResource *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _LeaseResource__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const spot_msgs::msg::LeaseResource *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _LeaseResource__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_LeaseResource(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _LeaseResource__callbacks = {
  "spot_msgs::msg",
  "LeaseResource",
  _LeaseResource__cdr_serialize,
  _LeaseResource__cdr_deserialize,
  _LeaseResource__get_serialized_size,
  _LeaseResource__max_serialized_size
};

static rosidl_message_type_support_t _LeaseResource__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_LeaseResource__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace spot_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_spot_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<spot_msgs::msg::LeaseResource>()
{
  return &spot_msgs::msg::typesupport_fastrtps_cpp::_LeaseResource__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, spot_msgs, msg, LeaseResource)() {
  return &spot_msgs::msg::typesupport_fastrtps_cpp::_LeaseResource__handle;
}

#ifdef __cplusplus
}
#endif
