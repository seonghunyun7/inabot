// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from inabot_msgs:msg/ObstacleStatus.idl
// generated code does not contain a copyright notice

#ifndef INABOT_MSGS__MSG__DETAIL__OBSTACLE_STATUS__FUNCTIONS_H_
#define INABOT_MSGS__MSG__DETAIL__OBSTACLE_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "inabot_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "inabot_msgs/msg/detail/obstacle_status__struct.h"

/// Initialize msg/ObstacleStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * inabot_msgs__msg__ObstacleStatus
 * )) before or use
 * inabot_msgs__msg__ObstacleStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_inabot_msgs
bool
inabot_msgs__msg__ObstacleStatus__init(inabot_msgs__msg__ObstacleStatus * msg);

/// Finalize msg/ObstacleStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_inabot_msgs
void
inabot_msgs__msg__ObstacleStatus__fini(inabot_msgs__msg__ObstacleStatus * msg);

/// Create msg/ObstacleStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * inabot_msgs__msg__ObstacleStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_inabot_msgs
inabot_msgs__msg__ObstacleStatus *
inabot_msgs__msg__ObstacleStatus__create();

/// Destroy msg/ObstacleStatus message.
/**
 * It calls
 * inabot_msgs__msg__ObstacleStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_inabot_msgs
void
inabot_msgs__msg__ObstacleStatus__destroy(inabot_msgs__msg__ObstacleStatus * msg);

/// Check for msg/ObstacleStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_inabot_msgs
bool
inabot_msgs__msg__ObstacleStatus__are_equal(const inabot_msgs__msg__ObstacleStatus * lhs, const inabot_msgs__msg__ObstacleStatus * rhs);

/// Copy a msg/ObstacleStatus message.
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
ROSIDL_GENERATOR_C_PUBLIC_inabot_msgs
bool
inabot_msgs__msg__ObstacleStatus__copy(
  const inabot_msgs__msg__ObstacleStatus * input,
  inabot_msgs__msg__ObstacleStatus * output);

/// Initialize array of msg/ObstacleStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * inabot_msgs__msg__ObstacleStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_inabot_msgs
bool
inabot_msgs__msg__ObstacleStatus__Sequence__init(inabot_msgs__msg__ObstacleStatus__Sequence * array, size_t size);

/// Finalize array of msg/ObstacleStatus messages.
/**
 * It calls
 * inabot_msgs__msg__ObstacleStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_inabot_msgs
void
inabot_msgs__msg__ObstacleStatus__Sequence__fini(inabot_msgs__msg__ObstacleStatus__Sequence * array);

/// Create array of msg/ObstacleStatus messages.
/**
 * It allocates the memory for the array and calls
 * inabot_msgs__msg__ObstacleStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_inabot_msgs
inabot_msgs__msg__ObstacleStatus__Sequence *
inabot_msgs__msg__ObstacleStatus__Sequence__create(size_t size);

/// Destroy array of msg/ObstacleStatus messages.
/**
 * It calls
 * inabot_msgs__msg__ObstacleStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_inabot_msgs
void
inabot_msgs__msg__ObstacleStatus__Sequence__destroy(inabot_msgs__msg__ObstacleStatus__Sequence * array);

/// Check for msg/ObstacleStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_inabot_msgs
bool
inabot_msgs__msg__ObstacleStatus__Sequence__are_equal(const inabot_msgs__msg__ObstacleStatus__Sequence * lhs, const inabot_msgs__msg__ObstacleStatus__Sequence * rhs);

/// Copy an array of msg/ObstacleStatus messages.
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
ROSIDL_GENERATOR_C_PUBLIC_inabot_msgs
bool
inabot_msgs__msg__ObstacleStatus__Sequence__copy(
  const inabot_msgs__msg__ObstacleStatus__Sequence * input,
  inabot_msgs__msg__ObstacleStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // INABOT_MSGS__MSG__DETAIL__OBSTACLE_STATUS__FUNCTIONS_H_
