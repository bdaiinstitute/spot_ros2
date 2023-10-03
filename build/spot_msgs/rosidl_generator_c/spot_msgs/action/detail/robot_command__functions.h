// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from spot_msgs:action/RobotCommand.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__ACTION__DETAIL__ROBOT_COMMAND__FUNCTIONS_H_
#define SPOT_MSGS__ACTION__DETAIL__ROBOT_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "spot_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "spot_msgs/action/detail/robot_command__struct.h"

/// Initialize action/RobotCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * spot_msgs__action__RobotCommand_Goal
 * )) before or use
 * spot_msgs__action__RobotCommand_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_Goal__init(spot_msgs__action__RobotCommand_Goal * msg);

/// Finalize action/RobotCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_Goal__fini(spot_msgs__action__RobotCommand_Goal * msg);

/// Create action/RobotCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * spot_msgs__action__RobotCommand_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
spot_msgs__action__RobotCommand_Goal *
spot_msgs__action__RobotCommand_Goal__create();

/// Destroy action/RobotCommand message.
/**
 * It calls
 * spot_msgs__action__RobotCommand_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_Goal__destroy(spot_msgs__action__RobotCommand_Goal * msg);

/// Check for action/RobotCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_Goal__are_equal(const spot_msgs__action__RobotCommand_Goal * lhs, const spot_msgs__action__RobotCommand_Goal * rhs);

/// Copy a action/RobotCommand message.
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
spot_msgs__action__RobotCommand_Goal__copy(
  const spot_msgs__action__RobotCommand_Goal * input,
  spot_msgs__action__RobotCommand_Goal * output);

/// Initialize array of action/RobotCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * spot_msgs__action__RobotCommand_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_Goal__Sequence__init(spot_msgs__action__RobotCommand_Goal__Sequence * array, size_t size);

/// Finalize array of action/RobotCommand messages.
/**
 * It calls
 * spot_msgs__action__RobotCommand_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_Goal__Sequence__fini(spot_msgs__action__RobotCommand_Goal__Sequence * array);

/// Create array of action/RobotCommand messages.
/**
 * It allocates the memory for the array and calls
 * spot_msgs__action__RobotCommand_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
spot_msgs__action__RobotCommand_Goal__Sequence *
spot_msgs__action__RobotCommand_Goal__Sequence__create(size_t size);

/// Destroy array of action/RobotCommand messages.
/**
 * It calls
 * spot_msgs__action__RobotCommand_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_Goal__Sequence__destroy(spot_msgs__action__RobotCommand_Goal__Sequence * array);

/// Check for action/RobotCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_Goal__Sequence__are_equal(const spot_msgs__action__RobotCommand_Goal__Sequence * lhs, const spot_msgs__action__RobotCommand_Goal__Sequence * rhs);

/// Copy an array of action/RobotCommand messages.
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
spot_msgs__action__RobotCommand_Goal__Sequence__copy(
  const spot_msgs__action__RobotCommand_Goal__Sequence * input,
  spot_msgs__action__RobotCommand_Goal__Sequence * output);

/// Initialize action/RobotCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * spot_msgs__action__RobotCommand_Result
 * )) before or use
 * spot_msgs__action__RobotCommand_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_Result__init(spot_msgs__action__RobotCommand_Result * msg);

/// Finalize action/RobotCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_Result__fini(spot_msgs__action__RobotCommand_Result * msg);

/// Create action/RobotCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * spot_msgs__action__RobotCommand_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
spot_msgs__action__RobotCommand_Result *
spot_msgs__action__RobotCommand_Result__create();

/// Destroy action/RobotCommand message.
/**
 * It calls
 * spot_msgs__action__RobotCommand_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_Result__destroy(spot_msgs__action__RobotCommand_Result * msg);

/// Check for action/RobotCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_Result__are_equal(const spot_msgs__action__RobotCommand_Result * lhs, const spot_msgs__action__RobotCommand_Result * rhs);

/// Copy a action/RobotCommand message.
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
spot_msgs__action__RobotCommand_Result__copy(
  const spot_msgs__action__RobotCommand_Result * input,
  spot_msgs__action__RobotCommand_Result * output);

/// Initialize array of action/RobotCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * spot_msgs__action__RobotCommand_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_Result__Sequence__init(spot_msgs__action__RobotCommand_Result__Sequence * array, size_t size);

/// Finalize array of action/RobotCommand messages.
/**
 * It calls
 * spot_msgs__action__RobotCommand_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_Result__Sequence__fini(spot_msgs__action__RobotCommand_Result__Sequence * array);

/// Create array of action/RobotCommand messages.
/**
 * It allocates the memory for the array and calls
 * spot_msgs__action__RobotCommand_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
spot_msgs__action__RobotCommand_Result__Sequence *
spot_msgs__action__RobotCommand_Result__Sequence__create(size_t size);

/// Destroy array of action/RobotCommand messages.
/**
 * It calls
 * spot_msgs__action__RobotCommand_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_Result__Sequence__destroy(spot_msgs__action__RobotCommand_Result__Sequence * array);

/// Check for action/RobotCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_Result__Sequence__are_equal(const spot_msgs__action__RobotCommand_Result__Sequence * lhs, const spot_msgs__action__RobotCommand_Result__Sequence * rhs);

/// Copy an array of action/RobotCommand messages.
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
spot_msgs__action__RobotCommand_Result__Sequence__copy(
  const spot_msgs__action__RobotCommand_Result__Sequence * input,
  spot_msgs__action__RobotCommand_Result__Sequence * output);

/// Initialize action/RobotCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * spot_msgs__action__RobotCommand_Feedback
 * )) before or use
 * spot_msgs__action__RobotCommand_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_Feedback__init(spot_msgs__action__RobotCommand_Feedback * msg);

/// Finalize action/RobotCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_Feedback__fini(spot_msgs__action__RobotCommand_Feedback * msg);

/// Create action/RobotCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * spot_msgs__action__RobotCommand_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
spot_msgs__action__RobotCommand_Feedback *
spot_msgs__action__RobotCommand_Feedback__create();

/// Destroy action/RobotCommand message.
/**
 * It calls
 * spot_msgs__action__RobotCommand_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_Feedback__destroy(spot_msgs__action__RobotCommand_Feedback * msg);

/// Check for action/RobotCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_Feedback__are_equal(const spot_msgs__action__RobotCommand_Feedback * lhs, const spot_msgs__action__RobotCommand_Feedback * rhs);

/// Copy a action/RobotCommand message.
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
spot_msgs__action__RobotCommand_Feedback__copy(
  const spot_msgs__action__RobotCommand_Feedback * input,
  spot_msgs__action__RobotCommand_Feedback * output);

/// Initialize array of action/RobotCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * spot_msgs__action__RobotCommand_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_Feedback__Sequence__init(spot_msgs__action__RobotCommand_Feedback__Sequence * array, size_t size);

/// Finalize array of action/RobotCommand messages.
/**
 * It calls
 * spot_msgs__action__RobotCommand_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_Feedback__Sequence__fini(spot_msgs__action__RobotCommand_Feedback__Sequence * array);

/// Create array of action/RobotCommand messages.
/**
 * It allocates the memory for the array and calls
 * spot_msgs__action__RobotCommand_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
spot_msgs__action__RobotCommand_Feedback__Sequence *
spot_msgs__action__RobotCommand_Feedback__Sequence__create(size_t size);

/// Destroy array of action/RobotCommand messages.
/**
 * It calls
 * spot_msgs__action__RobotCommand_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_Feedback__Sequence__destroy(spot_msgs__action__RobotCommand_Feedback__Sequence * array);

/// Check for action/RobotCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_Feedback__Sequence__are_equal(const spot_msgs__action__RobotCommand_Feedback__Sequence * lhs, const spot_msgs__action__RobotCommand_Feedback__Sequence * rhs);

/// Copy an array of action/RobotCommand messages.
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
spot_msgs__action__RobotCommand_Feedback__Sequence__copy(
  const spot_msgs__action__RobotCommand_Feedback__Sequence * input,
  spot_msgs__action__RobotCommand_Feedback__Sequence * output);

/// Initialize action/RobotCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * spot_msgs__action__RobotCommand_SendGoal_Request
 * )) before or use
 * spot_msgs__action__RobotCommand_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_SendGoal_Request__init(spot_msgs__action__RobotCommand_SendGoal_Request * msg);

/// Finalize action/RobotCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_SendGoal_Request__fini(spot_msgs__action__RobotCommand_SendGoal_Request * msg);

/// Create action/RobotCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * spot_msgs__action__RobotCommand_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
spot_msgs__action__RobotCommand_SendGoal_Request *
spot_msgs__action__RobotCommand_SendGoal_Request__create();

/// Destroy action/RobotCommand message.
/**
 * It calls
 * spot_msgs__action__RobotCommand_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_SendGoal_Request__destroy(spot_msgs__action__RobotCommand_SendGoal_Request * msg);

/// Check for action/RobotCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_SendGoal_Request__are_equal(const spot_msgs__action__RobotCommand_SendGoal_Request * lhs, const spot_msgs__action__RobotCommand_SendGoal_Request * rhs);

/// Copy a action/RobotCommand message.
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
spot_msgs__action__RobotCommand_SendGoal_Request__copy(
  const spot_msgs__action__RobotCommand_SendGoal_Request * input,
  spot_msgs__action__RobotCommand_SendGoal_Request * output);

/// Initialize array of action/RobotCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * spot_msgs__action__RobotCommand_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_SendGoal_Request__Sequence__init(spot_msgs__action__RobotCommand_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/RobotCommand messages.
/**
 * It calls
 * spot_msgs__action__RobotCommand_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_SendGoal_Request__Sequence__fini(spot_msgs__action__RobotCommand_SendGoal_Request__Sequence * array);

/// Create array of action/RobotCommand messages.
/**
 * It allocates the memory for the array and calls
 * spot_msgs__action__RobotCommand_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
spot_msgs__action__RobotCommand_SendGoal_Request__Sequence *
spot_msgs__action__RobotCommand_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/RobotCommand messages.
/**
 * It calls
 * spot_msgs__action__RobotCommand_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_SendGoal_Request__Sequence__destroy(spot_msgs__action__RobotCommand_SendGoal_Request__Sequence * array);

/// Check for action/RobotCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_SendGoal_Request__Sequence__are_equal(const spot_msgs__action__RobotCommand_SendGoal_Request__Sequence * lhs, const spot_msgs__action__RobotCommand_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/RobotCommand messages.
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
spot_msgs__action__RobotCommand_SendGoal_Request__Sequence__copy(
  const spot_msgs__action__RobotCommand_SendGoal_Request__Sequence * input,
  spot_msgs__action__RobotCommand_SendGoal_Request__Sequence * output);

/// Initialize action/RobotCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * spot_msgs__action__RobotCommand_SendGoal_Response
 * )) before or use
 * spot_msgs__action__RobotCommand_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_SendGoal_Response__init(spot_msgs__action__RobotCommand_SendGoal_Response * msg);

/// Finalize action/RobotCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_SendGoal_Response__fini(spot_msgs__action__RobotCommand_SendGoal_Response * msg);

/// Create action/RobotCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * spot_msgs__action__RobotCommand_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
spot_msgs__action__RobotCommand_SendGoal_Response *
spot_msgs__action__RobotCommand_SendGoal_Response__create();

/// Destroy action/RobotCommand message.
/**
 * It calls
 * spot_msgs__action__RobotCommand_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_SendGoal_Response__destroy(spot_msgs__action__RobotCommand_SendGoal_Response * msg);

/// Check for action/RobotCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_SendGoal_Response__are_equal(const spot_msgs__action__RobotCommand_SendGoal_Response * lhs, const spot_msgs__action__RobotCommand_SendGoal_Response * rhs);

/// Copy a action/RobotCommand message.
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
spot_msgs__action__RobotCommand_SendGoal_Response__copy(
  const spot_msgs__action__RobotCommand_SendGoal_Response * input,
  spot_msgs__action__RobotCommand_SendGoal_Response * output);

/// Initialize array of action/RobotCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * spot_msgs__action__RobotCommand_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_SendGoal_Response__Sequence__init(spot_msgs__action__RobotCommand_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/RobotCommand messages.
/**
 * It calls
 * spot_msgs__action__RobotCommand_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_SendGoal_Response__Sequence__fini(spot_msgs__action__RobotCommand_SendGoal_Response__Sequence * array);

/// Create array of action/RobotCommand messages.
/**
 * It allocates the memory for the array and calls
 * spot_msgs__action__RobotCommand_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
spot_msgs__action__RobotCommand_SendGoal_Response__Sequence *
spot_msgs__action__RobotCommand_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/RobotCommand messages.
/**
 * It calls
 * spot_msgs__action__RobotCommand_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_SendGoal_Response__Sequence__destroy(spot_msgs__action__RobotCommand_SendGoal_Response__Sequence * array);

/// Check for action/RobotCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_SendGoal_Response__Sequence__are_equal(const spot_msgs__action__RobotCommand_SendGoal_Response__Sequence * lhs, const spot_msgs__action__RobotCommand_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/RobotCommand messages.
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
spot_msgs__action__RobotCommand_SendGoal_Response__Sequence__copy(
  const spot_msgs__action__RobotCommand_SendGoal_Response__Sequence * input,
  spot_msgs__action__RobotCommand_SendGoal_Response__Sequence * output);

/// Initialize action/RobotCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * spot_msgs__action__RobotCommand_GetResult_Request
 * )) before or use
 * spot_msgs__action__RobotCommand_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_GetResult_Request__init(spot_msgs__action__RobotCommand_GetResult_Request * msg);

/// Finalize action/RobotCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_GetResult_Request__fini(spot_msgs__action__RobotCommand_GetResult_Request * msg);

/// Create action/RobotCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * spot_msgs__action__RobotCommand_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
spot_msgs__action__RobotCommand_GetResult_Request *
spot_msgs__action__RobotCommand_GetResult_Request__create();

/// Destroy action/RobotCommand message.
/**
 * It calls
 * spot_msgs__action__RobotCommand_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_GetResult_Request__destroy(spot_msgs__action__RobotCommand_GetResult_Request * msg);

/// Check for action/RobotCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_GetResult_Request__are_equal(const spot_msgs__action__RobotCommand_GetResult_Request * lhs, const spot_msgs__action__RobotCommand_GetResult_Request * rhs);

/// Copy a action/RobotCommand message.
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
spot_msgs__action__RobotCommand_GetResult_Request__copy(
  const spot_msgs__action__RobotCommand_GetResult_Request * input,
  spot_msgs__action__RobotCommand_GetResult_Request * output);

/// Initialize array of action/RobotCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * spot_msgs__action__RobotCommand_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_GetResult_Request__Sequence__init(spot_msgs__action__RobotCommand_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/RobotCommand messages.
/**
 * It calls
 * spot_msgs__action__RobotCommand_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_GetResult_Request__Sequence__fini(spot_msgs__action__RobotCommand_GetResult_Request__Sequence * array);

/// Create array of action/RobotCommand messages.
/**
 * It allocates the memory for the array and calls
 * spot_msgs__action__RobotCommand_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
spot_msgs__action__RobotCommand_GetResult_Request__Sequence *
spot_msgs__action__RobotCommand_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/RobotCommand messages.
/**
 * It calls
 * spot_msgs__action__RobotCommand_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_GetResult_Request__Sequence__destroy(spot_msgs__action__RobotCommand_GetResult_Request__Sequence * array);

/// Check for action/RobotCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_GetResult_Request__Sequence__are_equal(const spot_msgs__action__RobotCommand_GetResult_Request__Sequence * lhs, const spot_msgs__action__RobotCommand_GetResult_Request__Sequence * rhs);

/// Copy an array of action/RobotCommand messages.
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
spot_msgs__action__RobotCommand_GetResult_Request__Sequence__copy(
  const spot_msgs__action__RobotCommand_GetResult_Request__Sequence * input,
  spot_msgs__action__RobotCommand_GetResult_Request__Sequence * output);

/// Initialize action/RobotCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * spot_msgs__action__RobotCommand_GetResult_Response
 * )) before or use
 * spot_msgs__action__RobotCommand_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_GetResult_Response__init(spot_msgs__action__RobotCommand_GetResult_Response * msg);

/// Finalize action/RobotCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_GetResult_Response__fini(spot_msgs__action__RobotCommand_GetResult_Response * msg);

/// Create action/RobotCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * spot_msgs__action__RobotCommand_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
spot_msgs__action__RobotCommand_GetResult_Response *
spot_msgs__action__RobotCommand_GetResult_Response__create();

/// Destroy action/RobotCommand message.
/**
 * It calls
 * spot_msgs__action__RobotCommand_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_GetResult_Response__destroy(spot_msgs__action__RobotCommand_GetResult_Response * msg);

/// Check for action/RobotCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_GetResult_Response__are_equal(const spot_msgs__action__RobotCommand_GetResult_Response * lhs, const spot_msgs__action__RobotCommand_GetResult_Response * rhs);

/// Copy a action/RobotCommand message.
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
spot_msgs__action__RobotCommand_GetResult_Response__copy(
  const spot_msgs__action__RobotCommand_GetResult_Response * input,
  spot_msgs__action__RobotCommand_GetResult_Response * output);

/// Initialize array of action/RobotCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * spot_msgs__action__RobotCommand_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_GetResult_Response__Sequence__init(spot_msgs__action__RobotCommand_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/RobotCommand messages.
/**
 * It calls
 * spot_msgs__action__RobotCommand_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_GetResult_Response__Sequence__fini(spot_msgs__action__RobotCommand_GetResult_Response__Sequence * array);

/// Create array of action/RobotCommand messages.
/**
 * It allocates the memory for the array and calls
 * spot_msgs__action__RobotCommand_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
spot_msgs__action__RobotCommand_GetResult_Response__Sequence *
spot_msgs__action__RobotCommand_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/RobotCommand messages.
/**
 * It calls
 * spot_msgs__action__RobotCommand_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_GetResult_Response__Sequence__destroy(spot_msgs__action__RobotCommand_GetResult_Response__Sequence * array);

/// Check for action/RobotCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_GetResult_Response__Sequence__are_equal(const spot_msgs__action__RobotCommand_GetResult_Response__Sequence * lhs, const spot_msgs__action__RobotCommand_GetResult_Response__Sequence * rhs);

/// Copy an array of action/RobotCommand messages.
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
spot_msgs__action__RobotCommand_GetResult_Response__Sequence__copy(
  const spot_msgs__action__RobotCommand_GetResult_Response__Sequence * input,
  spot_msgs__action__RobotCommand_GetResult_Response__Sequence * output);

/// Initialize action/RobotCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * spot_msgs__action__RobotCommand_FeedbackMessage
 * )) before or use
 * spot_msgs__action__RobotCommand_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_FeedbackMessage__init(spot_msgs__action__RobotCommand_FeedbackMessage * msg);

/// Finalize action/RobotCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_FeedbackMessage__fini(spot_msgs__action__RobotCommand_FeedbackMessage * msg);

/// Create action/RobotCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * spot_msgs__action__RobotCommand_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
spot_msgs__action__RobotCommand_FeedbackMessage *
spot_msgs__action__RobotCommand_FeedbackMessage__create();

/// Destroy action/RobotCommand message.
/**
 * It calls
 * spot_msgs__action__RobotCommand_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_FeedbackMessage__destroy(spot_msgs__action__RobotCommand_FeedbackMessage * msg);

/// Check for action/RobotCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_FeedbackMessage__are_equal(const spot_msgs__action__RobotCommand_FeedbackMessage * lhs, const spot_msgs__action__RobotCommand_FeedbackMessage * rhs);

/// Copy a action/RobotCommand message.
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
spot_msgs__action__RobotCommand_FeedbackMessage__copy(
  const spot_msgs__action__RobotCommand_FeedbackMessage * input,
  spot_msgs__action__RobotCommand_FeedbackMessage * output);

/// Initialize array of action/RobotCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * spot_msgs__action__RobotCommand_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_FeedbackMessage__Sequence__init(spot_msgs__action__RobotCommand_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/RobotCommand messages.
/**
 * It calls
 * spot_msgs__action__RobotCommand_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_FeedbackMessage__Sequence__fini(spot_msgs__action__RobotCommand_FeedbackMessage__Sequence * array);

/// Create array of action/RobotCommand messages.
/**
 * It allocates the memory for the array and calls
 * spot_msgs__action__RobotCommand_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
spot_msgs__action__RobotCommand_FeedbackMessage__Sequence *
spot_msgs__action__RobotCommand_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/RobotCommand messages.
/**
 * It calls
 * spot_msgs__action__RobotCommand_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
void
spot_msgs__action__RobotCommand_FeedbackMessage__Sequence__destroy(spot_msgs__action__RobotCommand_FeedbackMessage__Sequence * array);

/// Check for action/RobotCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_spot_msgs
bool
spot_msgs__action__RobotCommand_FeedbackMessage__Sequence__are_equal(const spot_msgs__action__RobotCommand_FeedbackMessage__Sequence * lhs, const spot_msgs__action__RobotCommand_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/RobotCommand messages.
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
spot_msgs__action__RobotCommand_FeedbackMessage__Sequence__copy(
  const spot_msgs__action__RobotCommand_FeedbackMessage__Sequence * input,
  spot_msgs__action__RobotCommand_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__ACTION__DETAIL__ROBOT_COMMAND__FUNCTIONS_H_
