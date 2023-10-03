// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from spot_msgs:action/NavigateTo.idl
// generated code does not contain a copyright notice
#include "spot_msgs/action/detail/navigate_to__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `upload_path`
// Member `navigate_to`
// Member `initial_localization_waypoint`
#include "rosidl_runtime_c/string_functions.h"

bool
spot_msgs__action__NavigateTo_Goal__init(spot_msgs__action__NavigateTo_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // upload_path
  if (!rosidl_runtime_c__String__init(&msg->upload_path)) {
    spot_msgs__action__NavigateTo_Goal__fini(msg);
    return false;
  }
  // navigate_to
  if (!rosidl_runtime_c__String__init(&msg->navigate_to)) {
    spot_msgs__action__NavigateTo_Goal__fini(msg);
    return false;
  }
  // initial_localization_fiducial
  // initial_localization_waypoint
  if (!rosidl_runtime_c__String__init(&msg->initial_localization_waypoint)) {
    spot_msgs__action__NavigateTo_Goal__fini(msg);
    return false;
  }
  return true;
}

void
spot_msgs__action__NavigateTo_Goal__fini(spot_msgs__action__NavigateTo_Goal * msg)
{
  if (!msg) {
    return;
  }
  // upload_path
  rosidl_runtime_c__String__fini(&msg->upload_path);
  // navigate_to
  rosidl_runtime_c__String__fini(&msg->navigate_to);
  // initial_localization_fiducial
  // initial_localization_waypoint
  rosidl_runtime_c__String__fini(&msg->initial_localization_waypoint);
}

bool
spot_msgs__action__NavigateTo_Goal__are_equal(const spot_msgs__action__NavigateTo_Goal * lhs, const spot_msgs__action__NavigateTo_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // upload_path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->upload_path), &(rhs->upload_path)))
  {
    return false;
  }
  // navigate_to
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->navigate_to), &(rhs->navigate_to)))
  {
    return false;
  }
  // initial_localization_fiducial
  if (lhs->initial_localization_fiducial != rhs->initial_localization_fiducial) {
    return false;
  }
  // initial_localization_waypoint
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->initial_localization_waypoint), &(rhs->initial_localization_waypoint)))
  {
    return false;
  }
  return true;
}

bool
spot_msgs__action__NavigateTo_Goal__copy(
  const spot_msgs__action__NavigateTo_Goal * input,
  spot_msgs__action__NavigateTo_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // upload_path
  if (!rosidl_runtime_c__String__copy(
      &(input->upload_path), &(output->upload_path)))
  {
    return false;
  }
  // navigate_to
  if (!rosidl_runtime_c__String__copy(
      &(input->navigate_to), &(output->navigate_to)))
  {
    return false;
  }
  // initial_localization_fiducial
  output->initial_localization_fiducial = input->initial_localization_fiducial;
  // initial_localization_waypoint
  if (!rosidl_runtime_c__String__copy(
      &(input->initial_localization_waypoint), &(output->initial_localization_waypoint)))
  {
    return false;
  }
  return true;
}

spot_msgs__action__NavigateTo_Goal *
spot_msgs__action__NavigateTo_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_Goal * msg = (spot_msgs__action__NavigateTo_Goal *)allocator.allocate(sizeof(spot_msgs__action__NavigateTo_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(spot_msgs__action__NavigateTo_Goal));
  bool success = spot_msgs__action__NavigateTo_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
spot_msgs__action__NavigateTo_Goal__destroy(spot_msgs__action__NavigateTo_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    spot_msgs__action__NavigateTo_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
spot_msgs__action__NavigateTo_Goal__Sequence__init(spot_msgs__action__NavigateTo_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_Goal * data = NULL;

  if (size) {
    data = (spot_msgs__action__NavigateTo_Goal *)allocator.zero_allocate(size, sizeof(spot_msgs__action__NavigateTo_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = spot_msgs__action__NavigateTo_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        spot_msgs__action__NavigateTo_Goal__fini(&data[i - 1]);
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
spot_msgs__action__NavigateTo_Goal__Sequence__fini(spot_msgs__action__NavigateTo_Goal__Sequence * array)
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
      spot_msgs__action__NavigateTo_Goal__fini(&array->data[i]);
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

spot_msgs__action__NavigateTo_Goal__Sequence *
spot_msgs__action__NavigateTo_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_Goal__Sequence * array = (spot_msgs__action__NavigateTo_Goal__Sequence *)allocator.allocate(sizeof(spot_msgs__action__NavigateTo_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = spot_msgs__action__NavigateTo_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
spot_msgs__action__NavigateTo_Goal__Sequence__destroy(spot_msgs__action__NavigateTo_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    spot_msgs__action__NavigateTo_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
spot_msgs__action__NavigateTo_Goal__Sequence__are_equal(const spot_msgs__action__NavigateTo_Goal__Sequence * lhs, const spot_msgs__action__NavigateTo_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!spot_msgs__action__NavigateTo_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
spot_msgs__action__NavigateTo_Goal__Sequence__copy(
  const spot_msgs__action__NavigateTo_Goal__Sequence * input,
  spot_msgs__action__NavigateTo_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(spot_msgs__action__NavigateTo_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    spot_msgs__action__NavigateTo_Goal * data =
      (spot_msgs__action__NavigateTo_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!spot_msgs__action__NavigateTo_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          spot_msgs__action__NavigateTo_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!spot_msgs__action__NavigateTo_Goal__copy(
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
spot_msgs__action__NavigateTo_Result__init(spot_msgs__action__NavigateTo_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    spot_msgs__action__NavigateTo_Result__fini(msg);
    return false;
  }
  return true;
}

void
spot_msgs__action__NavigateTo_Result__fini(spot_msgs__action__NavigateTo_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
spot_msgs__action__NavigateTo_Result__are_equal(const spot_msgs__action__NavigateTo_Result * lhs, const spot_msgs__action__NavigateTo_Result * rhs)
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
spot_msgs__action__NavigateTo_Result__copy(
  const spot_msgs__action__NavigateTo_Result * input,
  spot_msgs__action__NavigateTo_Result * output)
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

spot_msgs__action__NavigateTo_Result *
spot_msgs__action__NavigateTo_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_Result * msg = (spot_msgs__action__NavigateTo_Result *)allocator.allocate(sizeof(spot_msgs__action__NavigateTo_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(spot_msgs__action__NavigateTo_Result));
  bool success = spot_msgs__action__NavigateTo_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
spot_msgs__action__NavigateTo_Result__destroy(spot_msgs__action__NavigateTo_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    spot_msgs__action__NavigateTo_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
spot_msgs__action__NavigateTo_Result__Sequence__init(spot_msgs__action__NavigateTo_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_Result * data = NULL;

  if (size) {
    data = (spot_msgs__action__NavigateTo_Result *)allocator.zero_allocate(size, sizeof(spot_msgs__action__NavigateTo_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = spot_msgs__action__NavigateTo_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        spot_msgs__action__NavigateTo_Result__fini(&data[i - 1]);
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
spot_msgs__action__NavigateTo_Result__Sequence__fini(spot_msgs__action__NavigateTo_Result__Sequence * array)
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
      spot_msgs__action__NavigateTo_Result__fini(&array->data[i]);
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

spot_msgs__action__NavigateTo_Result__Sequence *
spot_msgs__action__NavigateTo_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_Result__Sequence * array = (spot_msgs__action__NavigateTo_Result__Sequence *)allocator.allocate(sizeof(spot_msgs__action__NavigateTo_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = spot_msgs__action__NavigateTo_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
spot_msgs__action__NavigateTo_Result__Sequence__destroy(spot_msgs__action__NavigateTo_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    spot_msgs__action__NavigateTo_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
spot_msgs__action__NavigateTo_Result__Sequence__are_equal(const spot_msgs__action__NavigateTo_Result__Sequence * lhs, const spot_msgs__action__NavigateTo_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!spot_msgs__action__NavigateTo_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
spot_msgs__action__NavigateTo_Result__Sequence__copy(
  const spot_msgs__action__NavigateTo_Result__Sequence * input,
  spot_msgs__action__NavigateTo_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(spot_msgs__action__NavigateTo_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    spot_msgs__action__NavigateTo_Result * data =
      (spot_msgs__action__NavigateTo_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!spot_msgs__action__NavigateTo_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          spot_msgs__action__NavigateTo_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!spot_msgs__action__NavigateTo_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `waypoint_id`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
spot_msgs__action__NavigateTo_Feedback__init(spot_msgs__action__NavigateTo_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // waypoint_id
  if (!rosidl_runtime_c__String__init(&msg->waypoint_id)) {
    spot_msgs__action__NavigateTo_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
spot_msgs__action__NavigateTo_Feedback__fini(spot_msgs__action__NavigateTo_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // waypoint_id
  rosidl_runtime_c__String__fini(&msg->waypoint_id);
}

bool
spot_msgs__action__NavigateTo_Feedback__are_equal(const spot_msgs__action__NavigateTo_Feedback * lhs, const spot_msgs__action__NavigateTo_Feedback * rhs)
{
  if (!lhs || !rhs) {
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
spot_msgs__action__NavigateTo_Feedback__copy(
  const spot_msgs__action__NavigateTo_Feedback * input,
  spot_msgs__action__NavigateTo_Feedback * output)
{
  if (!input || !output) {
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

spot_msgs__action__NavigateTo_Feedback *
spot_msgs__action__NavigateTo_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_Feedback * msg = (spot_msgs__action__NavigateTo_Feedback *)allocator.allocate(sizeof(spot_msgs__action__NavigateTo_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(spot_msgs__action__NavigateTo_Feedback));
  bool success = spot_msgs__action__NavigateTo_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
spot_msgs__action__NavigateTo_Feedback__destroy(spot_msgs__action__NavigateTo_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    spot_msgs__action__NavigateTo_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
spot_msgs__action__NavigateTo_Feedback__Sequence__init(spot_msgs__action__NavigateTo_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_Feedback * data = NULL;

  if (size) {
    data = (spot_msgs__action__NavigateTo_Feedback *)allocator.zero_allocate(size, sizeof(spot_msgs__action__NavigateTo_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = spot_msgs__action__NavigateTo_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        spot_msgs__action__NavigateTo_Feedback__fini(&data[i - 1]);
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
spot_msgs__action__NavigateTo_Feedback__Sequence__fini(spot_msgs__action__NavigateTo_Feedback__Sequence * array)
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
      spot_msgs__action__NavigateTo_Feedback__fini(&array->data[i]);
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

spot_msgs__action__NavigateTo_Feedback__Sequence *
spot_msgs__action__NavigateTo_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_Feedback__Sequence * array = (spot_msgs__action__NavigateTo_Feedback__Sequence *)allocator.allocate(sizeof(spot_msgs__action__NavigateTo_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = spot_msgs__action__NavigateTo_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
spot_msgs__action__NavigateTo_Feedback__Sequence__destroy(spot_msgs__action__NavigateTo_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    spot_msgs__action__NavigateTo_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
spot_msgs__action__NavigateTo_Feedback__Sequence__are_equal(const spot_msgs__action__NavigateTo_Feedback__Sequence * lhs, const spot_msgs__action__NavigateTo_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!spot_msgs__action__NavigateTo_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
spot_msgs__action__NavigateTo_Feedback__Sequence__copy(
  const spot_msgs__action__NavigateTo_Feedback__Sequence * input,
  spot_msgs__action__NavigateTo_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(spot_msgs__action__NavigateTo_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    spot_msgs__action__NavigateTo_Feedback * data =
      (spot_msgs__action__NavigateTo_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!spot_msgs__action__NavigateTo_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          spot_msgs__action__NavigateTo_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!spot_msgs__action__NavigateTo_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "spot_msgs/action/detail/navigate_to__functions.h"

bool
spot_msgs__action__NavigateTo_SendGoal_Request__init(spot_msgs__action__NavigateTo_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    spot_msgs__action__NavigateTo_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!spot_msgs__action__NavigateTo_Goal__init(&msg->goal)) {
    spot_msgs__action__NavigateTo_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
spot_msgs__action__NavigateTo_SendGoal_Request__fini(spot_msgs__action__NavigateTo_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  spot_msgs__action__NavigateTo_Goal__fini(&msg->goal);
}

bool
spot_msgs__action__NavigateTo_SendGoal_Request__are_equal(const spot_msgs__action__NavigateTo_SendGoal_Request * lhs, const spot_msgs__action__NavigateTo_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!spot_msgs__action__NavigateTo_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
spot_msgs__action__NavigateTo_SendGoal_Request__copy(
  const spot_msgs__action__NavigateTo_SendGoal_Request * input,
  spot_msgs__action__NavigateTo_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!spot_msgs__action__NavigateTo_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

spot_msgs__action__NavigateTo_SendGoal_Request *
spot_msgs__action__NavigateTo_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_SendGoal_Request * msg = (spot_msgs__action__NavigateTo_SendGoal_Request *)allocator.allocate(sizeof(spot_msgs__action__NavigateTo_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(spot_msgs__action__NavigateTo_SendGoal_Request));
  bool success = spot_msgs__action__NavigateTo_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
spot_msgs__action__NavigateTo_SendGoal_Request__destroy(spot_msgs__action__NavigateTo_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    spot_msgs__action__NavigateTo_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
spot_msgs__action__NavigateTo_SendGoal_Request__Sequence__init(spot_msgs__action__NavigateTo_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_SendGoal_Request * data = NULL;

  if (size) {
    data = (spot_msgs__action__NavigateTo_SendGoal_Request *)allocator.zero_allocate(size, sizeof(spot_msgs__action__NavigateTo_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = spot_msgs__action__NavigateTo_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        spot_msgs__action__NavigateTo_SendGoal_Request__fini(&data[i - 1]);
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
spot_msgs__action__NavigateTo_SendGoal_Request__Sequence__fini(spot_msgs__action__NavigateTo_SendGoal_Request__Sequence * array)
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
      spot_msgs__action__NavigateTo_SendGoal_Request__fini(&array->data[i]);
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

spot_msgs__action__NavigateTo_SendGoal_Request__Sequence *
spot_msgs__action__NavigateTo_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_SendGoal_Request__Sequence * array = (spot_msgs__action__NavigateTo_SendGoal_Request__Sequence *)allocator.allocate(sizeof(spot_msgs__action__NavigateTo_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = spot_msgs__action__NavigateTo_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
spot_msgs__action__NavigateTo_SendGoal_Request__Sequence__destroy(spot_msgs__action__NavigateTo_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    spot_msgs__action__NavigateTo_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
spot_msgs__action__NavigateTo_SendGoal_Request__Sequence__are_equal(const spot_msgs__action__NavigateTo_SendGoal_Request__Sequence * lhs, const spot_msgs__action__NavigateTo_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!spot_msgs__action__NavigateTo_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
spot_msgs__action__NavigateTo_SendGoal_Request__Sequence__copy(
  const spot_msgs__action__NavigateTo_SendGoal_Request__Sequence * input,
  spot_msgs__action__NavigateTo_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(spot_msgs__action__NavigateTo_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    spot_msgs__action__NavigateTo_SendGoal_Request * data =
      (spot_msgs__action__NavigateTo_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!spot_msgs__action__NavigateTo_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          spot_msgs__action__NavigateTo_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!spot_msgs__action__NavigateTo_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
spot_msgs__action__NavigateTo_SendGoal_Response__init(spot_msgs__action__NavigateTo_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    spot_msgs__action__NavigateTo_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
spot_msgs__action__NavigateTo_SendGoal_Response__fini(spot_msgs__action__NavigateTo_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
spot_msgs__action__NavigateTo_SendGoal_Response__are_equal(const spot_msgs__action__NavigateTo_SendGoal_Response * lhs, const spot_msgs__action__NavigateTo_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
spot_msgs__action__NavigateTo_SendGoal_Response__copy(
  const spot_msgs__action__NavigateTo_SendGoal_Response * input,
  spot_msgs__action__NavigateTo_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

spot_msgs__action__NavigateTo_SendGoal_Response *
spot_msgs__action__NavigateTo_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_SendGoal_Response * msg = (spot_msgs__action__NavigateTo_SendGoal_Response *)allocator.allocate(sizeof(spot_msgs__action__NavigateTo_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(spot_msgs__action__NavigateTo_SendGoal_Response));
  bool success = spot_msgs__action__NavigateTo_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
spot_msgs__action__NavigateTo_SendGoal_Response__destroy(spot_msgs__action__NavigateTo_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    spot_msgs__action__NavigateTo_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
spot_msgs__action__NavigateTo_SendGoal_Response__Sequence__init(spot_msgs__action__NavigateTo_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_SendGoal_Response * data = NULL;

  if (size) {
    data = (spot_msgs__action__NavigateTo_SendGoal_Response *)allocator.zero_allocate(size, sizeof(spot_msgs__action__NavigateTo_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = spot_msgs__action__NavigateTo_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        spot_msgs__action__NavigateTo_SendGoal_Response__fini(&data[i - 1]);
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
spot_msgs__action__NavigateTo_SendGoal_Response__Sequence__fini(spot_msgs__action__NavigateTo_SendGoal_Response__Sequence * array)
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
      spot_msgs__action__NavigateTo_SendGoal_Response__fini(&array->data[i]);
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

spot_msgs__action__NavigateTo_SendGoal_Response__Sequence *
spot_msgs__action__NavigateTo_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_SendGoal_Response__Sequence * array = (spot_msgs__action__NavigateTo_SendGoal_Response__Sequence *)allocator.allocate(sizeof(spot_msgs__action__NavigateTo_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = spot_msgs__action__NavigateTo_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
spot_msgs__action__NavigateTo_SendGoal_Response__Sequence__destroy(spot_msgs__action__NavigateTo_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    spot_msgs__action__NavigateTo_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
spot_msgs__action__NavigateTo_SendGoal_Response__Sequence__are_equal(const spot_msgs__action__NavigateTo_SendGoal_Response__Sequence * lhs, const spot_msgs__action__NavigateTo_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!spot_msgs__action__NavigateTo_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
spot_msgs__action__NavigateTo_SendGoal_Response__Sequence__copy(
  const spot_msgs__action__NavigateTo_SendGoal_Response__Sequence * input,
  spot_msgs__action__NavigateTo_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(spot_msgs__action__NavigateTo_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    spot_msgs__action__NavigateTo_SendGoal_Response * data =
      (spot_msgs__action__NavigateTo_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!spot_msgs__action__NavigateTo_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          spot_msgs__action__NavigateTo_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!spot_msgs__action__NavigateTo_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
spot_msgs__action__NavigateTo_GetResult_Request__init(spot_msgs__action__NavigateTo_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    spot_msgs__action__NavigateTo_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
spot_msgs__action__NavigateTo_GetResult_Request__fini(spot_msgs__action__NavigateTo_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
spot_msgs__action__NavigateTo_GetResult_Request__are_equal(const spot_msgs__action__NavigateTo_GetResult_Request * lhs, const spot_msgs__action__NavigateTo_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
spot_msgs__action__NavigateTo_GetResult_Request__copy(
  const spot_msgs__action__NavigateTo_GetResult_Request * input,
  spot_msgs__action__NavigateTo_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

spot_msgs__action__NavigateTo_GetResult_Request *
spot_msgs__action__NavigateTo_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_GetResult_Request * msg = (spot_msgs__action__NavigateTo_GetResult_Request *)allocator.allocate(sizeof(spot_msgs__action__NavigateTo_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(spot_msgs__action__NavigateTo_GetResult_Request));
  bool success = spot_msgs__action__NavigateTo_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
spot_msgs__action__NavigateTo_GetResult_Request__destroy(spot_msgs__action__NavigateTo_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    spot_msgs__action__NavigateTo_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
spot_msgs__action__NavigateTo_GetResult_Request__Sequence__init(spot_msgs__action__NavigateTo_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_GetResult_Request * data = NULL;

  if (size) {
    data = (spot_msgs__action__NavigateTo_GetResult_Request *)allocator.zero_allocate(size, sizeof(spot_msgs__action__NavigateTo_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = spot_msgs__action__NavigateTo_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        spot_msgs__action__NavigateTo_GetResult_Request__fini(&data[i - 1]);
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
spot_msgs__action__NavigateTo_GetResult_Request__Sequence__fini(spot_msgs__action__NavigateTo_GetResult_Request__Sequence * array)
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
      spot_msgs__action__NavigateTo_GetResult_Request__fini(&array->data[i]);
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

spot_msgs__action__NavigateTo_GetResult_Request__Sequence *
spot_msgs__action__NavigateTo_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_GetResult_Request__Sequence * array = (spot_msgs__action__NavigateTo_GetResult_Request__Sequence *)allocator.allocate(sizeof(spot_msgs__action__NavigateTo_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = spot_msgs__action__NavigateTo_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
spot_msgs__action__NavigateTo_GetResult_Request__Sequence__destroy(spot_msgs__action__NavigateTo_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    spot_msgs__action__NavigateTo_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
spot_msgs__action__NavigateTo_GetResult_Request__Sequence__are_equal(const spot_msgs__action__NavigateTo_GetResult_Request__Sequence * lhs, const spot_msgs__action__NavigateTo_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!spot_msgs__action__NavigateTo_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
spot_msgs__action__NavigateTo_GetResult_Request__Sequence__copy(
  const spot_msgs__action__NavigateTo_GetResult_Request__Sequence * input,
  spot_msgs__action__NavigateTo_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(spot_msgs__action__NavigateTo_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    spot_msgs__action__NavigateTo_GetResult_Request * data =
      (spot_msgs__action__NavigateTo_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!spot_msgs__action__NavigateTo_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          spot_msgs__action__NavigateTo_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!spot_msgs__action__NavigateTo_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "spot_msgs/action/detail/navigate_to__functions.h"

bool
spot_msgs__action__NavigateTo_GetResult_Response__init(spot_msgs__action__NavigateTo_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!spot_msgs__action__NavigateTo_Result__init(&msg->result)) {
    spot_msgs__action__NavigateTo_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
spot_msgs__action__NavigateTo_GetResult_Response__fini(spot_msgs__action__NavigateTo_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  spot_msgs__action__NavigateTo_Result__fini(&msg->result);
}

bool
spot_msgs__action__NavigateTo_GetResult_Response__are_equal(const spot_msgs__action__NavigateTo_GetResult_Response * lhs, const spot_msgs__action__NavigateTo_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!spot_msgs__action__NavigateTo_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
spot_msgs__action__NavigateTo_GetResult_Response__copy(
  const spot_msgs__action__NavigateTo_GetResult_Response * input,
  spot_msgs__action__NavigateTo_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!spot_msgs__action__NavigateTo_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

spot_msgs__action__NavigateTo_GetResult_Response *
spot_msgs__action__NavigateTo_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_GetResult_Response * msg = (spot_msgs__action__NavigateTo_GetResult_Response *)allocator.allocate(sizeof(spot_msgs__action__NavigateTo_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(spot_msgs__action__NavigateTo_GetResult_Response));
  bool success = spot_msgs__action__NavigateTo_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
spot_msgs__action__NavigateTo_GetResult_Response__destroy(spot_msgs__action__NavigateTo_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    spot_msgs__action__NavigateTo_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
spot_msgs__action__NavigateTo_GetResult_Response__Sequence__init(spot_msgs__action__NavigateTo_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_GetResult_Response * data = NULL;

  if (size) {
    data = (spot_msgs__action__NavigateTo_GetResult_Response *)allocator.zero_allocate(size, sizeof(spot_msgs__action__NavigateTo_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = spot_msgs__action__NavigateTo_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        spot_msgs__action__NavigateTo_GetResult_Response__fini(&data[i - 1]);
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
spot_msgs__action__NavigateTo_GetResult_Response__Sequence__fini(spot_msgs__action__NavigateTo_GetResult_Response__Sequence * array)
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
      spot_msgs__action__NavigateTo_GetResult_Response__fini(&array->data[i]);
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

spot_msgs__action__NavigateTo_GetResult_Response__Sequence *
spot_msgs__action__NavigateTo_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_GetResult_Response__Sequence * array = (spot_msgs__action__NavigateTo_GetResult_Response__Sequence *)allocator.allocate(sizeof(spot_msgs__action__NavigateTo_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = spot_msgs__action__NavigateTo_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
spot_msgs__action__NavigateTo_GetResult_Response__Sequence__destroy(spot_msgs__action__NavigateTo_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    spot_msgs__action__NavigateTo_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
spot_msgs__action__NavigateTo_GetResult_Response__Sequence__are_equal(const spot_msgs__action__NavigateTo_GetResult_Response__Sequence * lhs, const spot_msgs__action__NavigateTo_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!spot_msgs__action__NavigateTo_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
spot_msgs__action__NavigateTo_GetResult_Response__Sequence__copy(
  const spot_msgs__action__NavigateTo_GetResult_Response__Sequence * input,
  spot_msgs__action__NavigateTo_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(spot_msgs__action__NavigateTo_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    spot_msgs__action__NavigateTo_GetResult_Response * data =
      (spot_msgs__action__NavigateTo_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!spot_msgs__action__NavigateTo_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          spot_msgs__action__NavigateTo_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!spot_msgs__action__NavigateTo_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "spot_msgs/action/detail/navigate_to__functions.h"

bool
spot_msgs__action__NavigateTo_FeedbackMessage__init(spot_msgs__action__NavigateTo_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    spot_msgs__action__NavigateTo_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!spot_msgs__action__NavigateTo_Feedback__init(&msg->feedback)) {
    spot_msgs__action__NavigateTo_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
spot_msgs__action__NavigateTo_FeedbackMessage__fini(spot_msgs__action__NavigateTo_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  spot_msgs__action__NavigateTo_Feedback__fini(&msg->feedback);
}

bool
spot_msgs__action__NavigateTo_FeedbackMessage__are_equal(const spot_msgs__action__NavigateTo_FeedbackMessage * lhs, const spot_msgs__action__NavigateTo_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!spot_msgs__action__NavigateTo_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
spot_msgs__action__NavigateTo_FeedbackMessage__copy(
  const spot_msgs__action__NavigateTo_FeedbackMessage * input,
  spot_msgs__action__NavigateTo_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!spot_msgs__action__NavigateTo_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

spot_msgs__action__NavigateTo_FeedbackMessage *
spot_msgs__action__NavigateTo_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_FeedbackMessage * msg = (spot_msgs__action__NavigateTo_FeedbackMessage *)allocator.allocate(sizeof(spot_msgs__action__NavigateTo_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(spot_msgs__action__NavigateTo_FeedbackMessage));
  bool success = spot_msgs__action__NavigateTo_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
spot_msgs__action__NavigateTo_FeedbackMessage__destroy(spot_msgs__action__NavigateTo_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    spot_msgs__action__NavigateTo_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
spot_msgs__action__NavigateTo_FeedbackMessage__Sequence__init(spot_msgs__action__NavigateTo_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_FeedbackMessage * data = NULL;

  if (size) {
    data = (spot_msgs__action__NavigateTo_FeedbackMessage *)allocator.zero_allocate(size, sizeof(spot_msgs__action__NavigateTo_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = spot_msgs__action__NavigateTo_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        spot_msgs__action__NavigateTo_FeedbackMessage__fini(&data[i - 1]);
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
spot_msgs__action__NavigateTo_FeedbackMessage__Sequence__fini(spot_msgs__action__NavigateTo_FeedbackMessage__Sequence * array)
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
      spot_msgs__action__NavigateTo_FeedbackMessage__fini(&array->data[i]);
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

spot_msgs__action__NavigateTo_FeedbackMessage__Sequence *
spot_msgs__action__NavigateTo_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__action__NavigateTo_FeedbackMessage__Sequence * array = (spot_msgs__action__NavigateTo_FeedbackMessage__Sequence *)allocator.allocate(sizeof(spot_msgs__action__NavigateTo_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = spot_msgs__action__NavigateTo_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
spot_msgs__action__NavigateTo_FeedbackMessage__Sequence__destroy(spot_msgs__action__NavigateTo_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    spot_msgs__action__NavigateTo_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
spot_msgs__action__NavigateTo_FeedbackMessage__Sequence__are_equal(const spot_msgs__action__NavigateTo_FeedbackMessage__Sequence * lhs, const spot_msgs__action__NavigateTo_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!spot_msgs__action__NavigateTo_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
spot_msgs__action__NavigateTo_FeedbackMessage__Sequence__copy(
  const spot_msgs__action__NavigateTo_FeedbackMessage__Sequence * input,
  spot_msgs__action__NavigateTo_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(spot_msgs__action__NavigateTo_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    spot_msgs__action__NavigateTo_FeedbackMessage * data =
      (spot_msgs__action__NavigateTo_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!spot_msgs__action__NavigateTo_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          spot_msgs__action__NavigateTo_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!spot_msgs__action__NavigateTo_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
