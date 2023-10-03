// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from spot_msgs:msg/PowerState.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__POWER_STATE__FUNCTIONS_H_
#define SPOT_MSGS__MSG__DETAIL__POWER_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "spot_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "spot_msgs/msg/detail/power_state__struct.h"

/// Initialize msg/PowerState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * spot_msgs__msg__PowerState
 * )) before or use
 * spot_msgs__msg__PowerState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__msg__PowerState__init(spot_msgs__msg__PowerState * msg);

/// Finalize msg/PowerState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__msg__PowerState__fini(spot_msgs__msg__PowerState * msg);

/// Create msg/PowerState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * spot_msgs__msg__PowerState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
spot_msgs__msg__PowerState *
spot_msgs__msg__PowerState__create();

/// Destroy msg/PowerState message.
/**
 * It calls
 * spot_msgs__msg__PowerState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__msg__PowerState__destroy(spot_msgs__msg__PowerState * msg);

/// Check for msg/PowerState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__msg__PowerState__are_equal(const spot_msgs__msg__PowerState * lhs, const spot_msgs__msg__PowerState * rhs);

/// Copy a msg/PowerState message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__msg__PowerState__copy(
  const spot_msgs__msg__PowerState * input,
  spot_msgs__msg__PowerState * output);

/// Initialize array of msg/PowerState messages.
/**
 * It allocates the memory for the number of elements and calls
 * spot_msgs__msg__PowerState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__msg__PowerState__Sequence__init(spot_msgs__msg__PowerState__Sequence * array, size_t size);

/// Finalize array of msg/PowerState messages.
/**
 * It calls
 * spot_msgs__msg__PowerState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__msg__PowerState__Sequence__fini(spot_msgs__msg__PowerState__Sequence * array);

/// Create array of msg/PowerState messages.
/**
 * It allocates the memory for the array and calls
 * spot_msgs__msg__PowerState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
spot_msgs__msg__PowerState__Sequence *
spot_msgs__msg__PowerState__Sequence__create(size_t size);

/// Destroy array of msg/PowerState messages.
/**
 * It calls
 * spot_msgs__msg__PowerState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__msg__PowerState__Sequence__destroy(spot_msgs__msg__PowerState__Sequence * array);

/// Check for msg/PowerState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__msg__PowerState__Sequence__are_equal(const spot_msgs__msg__PowerState__Sequence * lhs, const spot_msgs__msg__PowerState__Sequence * rhs);

/// Copy an array of msg/PowerState messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__msg__PowerState__Sequence__copy(
  const spot_msgs__msg__PowerState__Sequence * input,
  spot_msgs__msg__PowerState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__MSG__DETAIL__POWER_STATE__FUNCTIONS_H_
