// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from spot_msgs:msg/BehaviorFaultState.idl
// generated code does not contain a copyright notice
#include "spot_msgs/msg/detail/behavior_fault_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `faults`
#include "spot_msgs/msg/detail/behavior_fault__functions.h"

bool
spot_msgs__msg__BehaviorFaultState__init(spot_msgs__msg__BehaviorFaultState * msg)
{
  if (!msg) {
    return false;
  }
  // faults
  if (!spot_msgs__msg__BehaviorFault__Sequence__init(&msg->faults, 0)) {
    spot_msgs__msg__BehaviorFaultState__fini(msg);
    return false;
  }
  return true;
}

void
spot_msgs__msg__BehaviorFaultState__fini(spot_msgs__msg__BehaviorFaultState * msg)
{
  if (!msg) {
    return;
  }
  // faults
  spot_msgs__msg__BehaviorFault__Sequence__fini(&msg->faults);
}

bool
spot_msgs__msg__BehaviorFaultState__are_equal(const spot_msgs__msg__BehaviorFaultState * lhs, const spot_msgs__msg__BehaviorFaultState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // faults
  if (!spot_msgs__msg__BehaviorFault__Sequence__are_equal(
      &(lhs->faults), &(rhs->faults)))
  {
    return false;
  }
  return true;
}

bool
spot_msgs__msg__BehaviorFaultState__copy(
  const spot_msgs__msg__BehaviorFaultState * input,
  spot_msgs__msg__BehaviorFaultState * output)
{
  if (!input || !output) {
    return false;
  }
  // faults
  if (!spot_msgs__msg__BehaviorFault__Sequence__copy(
      &(input->faults), &(output->faults)))
  {
    return false;
  }
  return true;
}

spot_msgs__msg__BehaviorFaultState *
spot_msgs__msg__BehaviorFaultState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__msg__BehaviorFaultState * msg = (spot_msgs__msg__BehaviorFaultState *)allocator.allocate(sizeof(spot_msgs__msg__BehaviorFaultState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(spot_msgs__msg__BehaviorFaultState));
  bool success = spot_msgs__msg__BehaviorFaultState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
spot_msgs__msg__BehaviorFaultState__destroy(spot_msgs__msg__BehaviorFaultState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    spot_msgs__msg__BehaviorFaultState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
spot_msgs__msg__BehaviorFaultState__Sequence__init(spot_msgs__msg__BehaviorFaultState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__msg__BehaviorFaultState * data = NULL;

  if (size) {
    data = (spot_msgs__msg__BehaviorFaultState *)allocator.zero_allocate(size, sizeof(spot_msgs__msg__BehaviorFaultState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = spot_msgs__msg__BehaviorFaultState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        spot_msgs__msg__BehaviorFaultState__fini(&data[i - 1]);
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
spot_msgs__msg__BehaviorFaultState__Sequence__fini(spot_msgs__msg__BehaviorFaultState__Sequence * array)
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
      spot_msgs__msg__BehaviorFaultState__fini(&array->data[i]);
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

spot_msgs__msg__BehaviorFaultState__Sequence *
spot_msgs__msg__BehaviorFaultState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  spot_msgs__msg__BehaviorFaultState__Sequence * array = (spot_msgs__msg__BehaviorFaultState__Sequence *)allocator.allocate(sizeof(spot_msgs__msg__BehaviorFaultState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = spot_msgs__msg__BehaviorFaultState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
spot_msgs__msg__BehaviorFaultState__Sequence__destroy(spot_msgs__msg__BehaviorFaultState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    spot_msgs__msg__BehaviorFaultState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
spot_msgs__msg__BehaviorFaultState__Sequence__are_equal(const spot_msgs__msg__BehaviorFaultState__Sequence * lhs, const spot_msgs__msg__BehaviorFaultState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!spot_msgs__msg__BehaviorFaultState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
spot_msgs__msg__BehaviorFaultState__Sequence__copy(
  const spot_msgs__msg__BehaviorFaultState__Sequence * input,
  spot_msgs__msg__BehaviorFaultState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(spot_msgs__msg__BehaviorFaultState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    spot_msgs__msg__BehaviorFaultState * data =
      (spot_msgs__msg__BehaviorFaultState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!spot_msgs__msg__BehaviorFaultState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          spot_msgs__msg__BehaviorFaultState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!spot_msgs__msg__BehaviorFaultState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
