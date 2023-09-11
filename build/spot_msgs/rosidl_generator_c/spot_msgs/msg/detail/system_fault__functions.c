// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from spot_msgs:msg/SystemFault.idl
// generated code does not contain a copyright notice
#include "spot_msgs/msg/detail/system_fault__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `name`
// Member `error_message`
// Member `attributes`
#include "rosidl_runtime_c/string_functions.h"
// Member `duration`
#include "builtin_interfaces/msg/detail/duration__functions.h"

bool
spot_msgs__msg__SystemFault__init(spot_msgs__msg__SystemFault * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    spot_msgs__msg__SystemFault__fini(msg);
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    spot_msgs__msg__SystemFault__fini(msg);
    return false;
  }
  // duration
  if (!builtin_interfaces__msg__Duration__init(&msg->duration)) {
    spot_msgs__msg__SystemFault__fini(msg);
    return false;
  }
  // code
  // uid
  // error_message
  if (!rosidl_runtime_c__String__init(&msg->error_message)) {
    spot_msgs__msg__SystemFault__fini(msg);
    return false;
  }
  // attributes
  if (!rosidl_runtime_c__String__Sequence__init(&msg->attributes, 0)) {
    spot_msgs__msg__SystemFault__fini(msg);
    return false;
  }
  // severity
  return true;
}

void
spot_msgs__msg__SystemFault__fini(spot_msgs__msg__SystemFault * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // duration
  builtin_interfaces__msg__Duration__fini(&msg->duration);
  // code
  // uid
  // error_message
  rosidl_runtime_c__String__fini(&msg->error_message);
  // attributes
  rosidl_runtime_c__String__Sequence__fini(&msg->attributes);
  // severity
}

bool
spot_msgs__msg__SystemFault__are_equal(const spot_msgs__msg__SystemFault * lhs, const spot_msgs__msg__SystemFault * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // duration
  if (!builtin_interfaces__msg__Duration__are_equal(
      &(lhs->duration), &(rhs->duration)))
  {
    return false;
  }
  // code
  if (lhs->code != rhs->code) {
    return false;
  }
  // uid
  if (lhs->uid != rhs->uid) {
    return false;
  }
  // error_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error_message), &(rhs->error_message)))
  {
    return false;
  }
  // attributes
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->attributes), &(rhs->attributes)))
  {
    return false;
  }
  // severity
  if (lhs->severity != rhs->severity) {
    return false;
  }
  return true;
}

bool
spot_msgs__msg__SystemFault__copy(
  const spot_msgs__msg__SystemFault * input,
  spot_msgs__msg__SystemFault * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // duration
  if (!builtin_interfaces__msg__Duration__copy(
      &(input->duration), &(output->duration)))
  {
    return false;
  }
  // code
  output->code = input->code;
  // uid
  output->uid = input->uid;
  // error_message
  if (!rosidl_runtime_c__String__copy(
      &(input->error_message), &(output->error_message)))
  {
    return false;
  }
  // attributes
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->attributes), &(output->attributes)))
  {
    return false;
  }
  // severity
  output->severity = input->severity;
  return true;
}

spot_msgs__msg__SystemFault *
spot_msgs__msg__SystemFault__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__msg__SystemFault * msg = (spot_msgs__msg__SystemFault *)allocator.allocate(sizeof(spot_msgs__msg__SystemFault), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(spot_msgs__msg__SystemFault));
  bool success = spot_msgs__msg__SystemFault__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
spot_msgs__msg__SystemFault__destroy(spot_msgs__msg__SystemFault * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    spot_msgs__msg__SystemFault__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
spot_msgs__msg__SystemFault__Sequence__init(spot_msgs__msg__SystemFault__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__msg__SystemFault * data = NULL;

  if (size) {
    data = (spot_msgs__msg__SystemFault *)allocator.zero_allocate(size, sizeof(spot_msgs__msg__SystemFault), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = spot_msgs__msg__SystemFault__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        spot_msgs__msg__SystemFault__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
spot_msgs__msg__SystemFault__Sequence__fini(spot_msgs__msg__SystemFault__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      spot_msgs__msg__SystemFault__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

spot_msgs__msg__SystemFault__Sequence *
spot_msgs__msg__SystemFault__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__msg__SystemFault__Sequence * array = (spot_msgs__msg__SystemFault__Sequence *)allocator.allocate(sizeof(spot_msgs__msg__SystemFault__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = spot_msgs__msg__SystemFault__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
spot_msgs__msg__SystemFault__Sequence__destroy(spot_msgs__msg__SystemFault__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    spot_msgs__msg__SystemFault__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
spot_msgs__msg__SystemFault__Sequence__are_equal(const spot_msgs__msg__SystemFault__Sequence * lhs, const spot_msgs__msg__SystemFault__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!spot_msgs__msg__SystemFault__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
spot_msgs__msg__SystemFault__Sequence__copy(
  const spot_msgs__msg__SystemFault__Sequence * input,
  spot_msgs__msg__SystemFault__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(spot_msgs__msg__SystemFault);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    spot_msgs__msg__SystemFault * data =
      (spot_msgs__msg__SystemFault *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!spot_msgs__msg__SystemFault__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          spot_msgs__msg__SystemFault__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!spot_msgs__msg__SystemFault__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
