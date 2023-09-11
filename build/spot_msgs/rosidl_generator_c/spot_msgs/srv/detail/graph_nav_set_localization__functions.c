// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from spot_msgs:srv/GraphNavSetLocalization.idl
// generated code does not contain a copyright notice
#include "spot_msgs/srv/detail/graph_nav_set_localization__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `method`
// Member `waypoint_id`
#include "rosidl_runtime_c/string_functions.h"

bool
spot_msgs__srv__GraphNavSetLocalization_Request__init(spot_msgs__srv__GraphNavSetLocalization_Request * msg)
{
  if (!msg) {
    return false;
  }
  // method
  if (!rosidl_runtime_c__String__init(&msg->method)) {
    spot_msgs__srv__GraphNavSetLocalization_Request__fini(msg);
    return false;
  }
  // waypoint_id
  if (!rosidl_runtime_c__String__init(&msg->waypoint_id)) {
    spot_msgs__srv__GraphNavSetLocalization_Request__fini(msg);
    return false;
  }
  return true;
}

void
spot_msgs__srv__GraphNavSetLocalization_Request__fini(spot_msgs__srv__GraphNavSetLocalization_Request * msg)
{
  if (!msg) {
    return;
  }
  // method
  rosidl_runtime_c__String__fini(&msg->method);
  // waypoint_id
  rosidl_runtime_c__String__fini(&msg->waypoint_id);
}

bool
spot_msgs__srv__GraphNavSetLocalization_Request__are_equal(const spot_msgs__srv__GraphNavSetLocalization_Request * lhs, const spot_msgs__srv__GraphNavSetLocalization_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // method
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->method), &(rhs->method)))
  {
    return false;
  }
  // waypoint_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->waypoint_id), &(rhs->waypoint_id)))
  {
    return false;
  }
  return true;
}

bool
spot_msgs__srv__GraphNavSetLocalization_Request__copy(
  const spot_msgs__srv__GraphNavSetLocalization_Request * input,
  spot_msgs__srv__GraphNavSetLocalization_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // method
  if (!rosidl_runtime_c__String__copy(
      &(input->method), &(output->method)))
  {
    return false;
  }
  // waypoint_id
  if (!rosidl_runtime_c__String__copy(
      &(input->waypoint_id), &(output->waypoint_id)))
  {
    return false;
  }
  return true;
}

spot_msgs__srv__GraphNavSetLocalization_Request *
spot_msgs__srv__GraphNavSetLocalization_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__srv__GraphNavSetLocalization_Request * msg = (spot_msgs__srv__GraphNavSetLocalization_Request *)allocator.allocate(sizeof(spot_msgs__srv__GraphNavSetLocalization_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(spot_msgs__srv__GraphNavSetLocalization_Request));
  bool success = spot_msgs__srv__GraphNavSetLocalization_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
spot_msgs__srv__GraphNavSetLocalization_Request__destroy(spot_msgs__srv__GraphNavSetLocalization_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    spot_msgs__srv__GraphNavSetLocalization_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
spot_msgs__srv__GraphNavSetLocalization_Request__Sequence__init(spot_msgs__srv__GraphNavSetLocalization_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__srv__GraphNavSetLocalization_Request * data = NULL;

  if (size) {
    data = (spot_msgs__srv__GraphNavSetLocalization_Request *)allocator.zero_allocate(size, sizeof(spot_msgs__srv__GraphNavSetLocalization_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = spot_msgs__srv__GraphNavSetLocalization_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        spot_msgs__srv__GraphNavSetLocalization_Request__fini(&data[i - 1]);
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
spot_msgs__srv__GraphNavSetLocalization_Request__Sequence__fini(spot_msgs__srv__GraphNavSetLocalization_Request__Sequence * array)
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
      spot_msgs__srv__GraphNavSetLocalization_Request__fini(&array->data[i]);
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

spot_msgs__srv__GraphNavSetLocalization_Request__Sequence *
spot_msgs__srv__GraphNavSetLocalization_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__srv__GraphNavSetLocalization_Request__Sequence * array = (spot_msgs__srv__GraphNavSetLocalization_Request__Sequence *)allocator.allocate(sizeof(spot_msgs__srv__GraphNavSetLocalization_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = spot_msgs__srv__GraphNavSetLocalization_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
spot_msgs__srv__GraphNavSetLocalization_Request__Sequence__destroy(spot_msgs__srv__GraphNavSetLocalization_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    spot_msgs__srv__GraphNavSetLocalization_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
spot_msgs__srv__GraphNavSetLocalization_Request__Sequence__are_equal(const spot_msgs__srv__GraphNavSetLocalization_Request__Sequence * lhs, const spot_msgs__srv__GraphNavSetLocalization_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!spot_msgs__srv__GraphNavSetLocalization_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
spot_msgs__srv__GraphNavSetLocalization_Request__Sequence__copy(
  const spot_msgs__srv__GraphNavSetLocalization_Request__Sequence * input,
  spot_msgs__srv__GraphNavSetLocalization_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(spot_msgs__srv__GraphNavSetLocalization_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    spot_msgs__srv__GraphNavSetLocalization_Request * data =
      (spot_msgs__srv__GraphNavSetLocalization_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!spot_msgs__srv__GraphNavSetLocalization_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          spot_msgs__srv__GraphNavSetLocalization_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!spot_msgs__srv__GraphNavSetLocalization_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
spot_msgs__srv__GraphNavSetLocalization_Response__init(spot_msgs__srv__GraphNavSetLocalization_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    spot_msgs__srv__GraphNavSetLocalization_Response__fini(msg);
    return false;
  }
  return true;
}

void
spot_msgs__srv__GraphNavSetLocalization_Response__fini(spot_msgs__srv__GraphNavSetLocalization_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
spot_msgs__srv__GraphNavSetLocalization_Response__are_equal(const spot_msgs__srv__GraphNavSetLocalization_Response * lhs, const spot_msgs__srv__GraphNavSetLocalization_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
spot_msgs__srv__GraphNavSetLocalization_Response__copy(
  const spot_msgs__srv__GraphNavSetLocalization_Response * input,
  spot_msgs__srv__GraphNavSetLocalization_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

spot_msgs__srv__GraphNavSetLocalization_Response *
spot_msgs__srv__GraphNavSetLocalization_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__srv__GraphNavSetLocalization_Response * msg = (spot_msgs__srv__GraphNavSetLocalization_Response *)allocator.allocate(sizeof(spot_msgs__srv__GraphNavSetLocalization_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(spot_msgs__srv__GraphNavSetLocalization_Response));
  bool success = spot_msgs__srv__GraphNavSetLocalization_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
spot_msgs__srv__GraphNavSetLocalization_Response__destroy(spot_msgs__srv__GraphNavSetLocalization_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    spot_msgs__srv__GraphNavSetLocalization_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
spot_msgs__srv__GraphNavSetLocalization_Response__Sequence__init(spot_msgs__srv__GraphNavSetLocalization_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__srv__GraphNavSetLocalization_Response * data = NULL;

  if (size) {
    data = (spot_msgs__srv__GraphNavSetLocalization_Response *)allocator.zero_allocate(size, sizeof(spot_msgs__srv__GraphNavSetLocalization_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = spot_msgs__srv__GraphNavSetLocalization_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        spot_msgs__srv__GraphNavSetLocalization_Response__fini(&data[i - 1]);
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
spot_msgs__srv__GraphNavSetLocalization_Response__Sequence__fini(spot_msgs__srv__GraphNavSetLocalization_Response__Sequence * array)
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
      spot_msgs__srv__GraphNavSetLocalization_Response__fini(&array->data[i]);
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

spot_msgs__srv__GraphNavSetLocalization_Response__Sequence *
spot_msgs__srv__GraphNavSetLocalization_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__srv__GraphNavSetLocalization_Response__Sequence * array = (spot_msgs__srv__GraphNavSetLocalization_Response__Sequence *)allocator.allocate(sizeof(spot_msgs__srv__GraphNavSetLocalization_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = spot_msgs__srv__GraphNavSetLocalization_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
spot_msgs__srv__GraphNavSetLocalization_Response__Sequence__destroy(spot_msgs__srv__GraphNavSetLocalization_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    spot_msgs__srv__GraphNavSetLocalization_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
spot_msgs__srv__GraphNavSetLocalization_Response__Sequence__are_equal(const spot_msgs__srv__GraphNavSetLocalization_Response__Sequence * lhs, const spot_msgs__srv__GraphNavSetLocalization_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!spot_msgs__srv__GraphNavSetLocalization_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
spot_msgs__srv__GraphNavSetLocalization_Response__Sequence__copy(
  const spot_msgs__srv__GraphNavSetLocalization_Response__Sequence * input,
  spot_msgs__srv__GraphNavSetLocalization_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(spot_msgs__srv__GraphNavSetLocalization_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    spot_msgs__srv__GraphNavSetLocalization_Response * data =
      (spot_msgs__srv__GraphNavSetLocalization_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!spot_msgs__srv__GraphNavSetLocalization_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          spot_msgs__srv__GraphNavSetLocalization_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!spot_msgs__srv__GraphNavSetLocalization_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
