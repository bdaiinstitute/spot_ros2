// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from spot_msgs:msg/FootStateArray.idl
// generated code does not contain a copyright notice
#include "spot_msgs/msg/detail/foot_state_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `states`
#include "spot_msgs/msg/detail/foot_state__functions.h"

bool
spot_msgs__msg__FootStateArray__init(spot_msgs__msg__FootStateArray * msg)
{
  if (!msg) {
    return false;
  }
  // states
  if (!spot_msgs__msg__FootState__Sequence__init(&msg->states, 0)) {
    spot_msgs__msg__FootStateArray__fini(msg);
    return false;
  }
  return true;
}

void
spot_msgs__msg__FootStateArray__fini(spot_msgs__msg__FootStateArray * msg)
{
  if (!msg) {
    return;
  }
  // states
  spot_msgs__msg__FootState__Sequence__fini(&msg->states);
}

bool
spot_msgs__msg__FootStateArray__are_equal(const spot_msgs__msg__FootStateArray * lhs, const spot_msgs__msg__FootStateArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // states
  if (!spot_msgs__msg__FootState__Sequence__are_equal(
      &(lhs->states), &(rhs->states)))
  {
    return false;
  }
  return true;
}

bool
spot_msgs__msg__FootStateArray__copy(
  const spot_msgs__msg__FootStateArray * input,
  spot_msgs__msg__FootStateArray * output)
{
  if (!input || !output) {
    return false;
  }
  // states
  if (!spot_msgs__msg__FootState__Sequence__copy(
      &(input->states), &(output->states)))
  {
    return false;
  }
  return true;
}

spot_msgs__msg__FootStateArray *
spot_msgs__msg__FootStateArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__msg__FootStateArray * msg = (spot_msgs__msg__FootStateArray *)allocator.allocate(sizeof(spot_msgs__msg__FootStateArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(spot_msgs__msg__FootStateArray));
  bool success = spot_msgs__msg__FootStateArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
spot_msgs__msg__FootStateArray__destroy(spot_msgs__msg__FootStateArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    spot_msgs__msg__FootStateArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
spot_msgs__msg__FootStateArray__Sequence__init(spot_msgs__msg__FootStateArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__msg__FootStateArray * data = NULL;

  if (size) {
    data = (spot_msgs__msg__FootStateArray *)allocator.zero_allocate(size, sizeof(spot_msgs__msg__FootStateArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = spot_msgs__msg__FootStateArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        spot_msgs__msg__FootStateArray__fini(&data[i - 1]);
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
spot_msgs__msg__FootStateArray__Sequence__fini(spot_msgs__msg__FootStateArray__Sequence * array)
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
      spot_msgs__msg__FootStateArray__fini(&array->data[i]);
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

spot_msgs__msg__FootStateArray__Sequence *
spot_msgs__msg__FootStateArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__msg__FootStateArray__Sequence * array = (spot_msgs__msg__FootStateArray__Sequence *)allocator.allocate(sizeof(spot_msgs__msg__FootStateArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = spot_msgs__msg__FootStateArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
spot_msgs__msg__FootStateArray__Sequence__destroy(spot_msgs__msg__FootStateArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    spot_msgs__msg__FootStateArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
spot_msgs__msg__FootStateArray__Sequence__are_equal(const spot_msgs__msg__FootStateArray__Sequence * lhs, const spot_msgs__msg__FootStateArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!spot_msgs__msg__FootStateArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
spot_msgs__msg__FootStateArray__Sequence__copy(
  const spot_msgs__msg__FootStateArray__Sequence * input,
  spot_msgs__msg__FootStateArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(spot_msgs__msg__FootStateArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    spot_msgs__msg__FootStateArray * data =
      (spot_msgs__msg__FootStateArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!spot_msgs__msg__FootStateArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          spot_msgs__msg__FootStateArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!spot_msgs__msg__FootStateArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
