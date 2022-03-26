// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from vesc_msgs:msg/VescState.idl
// generated code does not contain a copyright notice

#ifndef VESC_MSGS__MSG__DETAIL__VESC_STATE__FUNCTIONS_H_
#define VESC_MSGS__MSG__DETAIL__VESC_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "vesc_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "vesc_msgs/msg/detail/vesc_state__struct.h"

/// Initialize msg/VescState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * vesc_msgs__msg__VescState
 * )) before or use
 * vesc_msgs__msg__VescState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
bool
vesc_msgs__msg__VescState__init(vesc_msgs__msg__VescState * msg);

/// Finalize msg/VescState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
void
vesc_msgs__msg__VescState__fini(vesc_msgs__msg__VescState * msg);

/// Create msg/VescState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * vesc_msgs__msg__VescState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
vesc_msgs__msg__VescState *
vesc_msgs__msg__VescState__create();

/// Destroy msg/VescState message.
/**
 * It calls
 * vesc_msgs__msg__VescState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
void
vesc_msgs__msg__VescState__destroy(vesc_msgs__msg__VescState * msg);


/// Initialize array of msg/VescState messages.
/**
 * It allocates the memory for the number of elements and calls
 * vesc_msgs__msg__VescState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
bool
vesc_msgs__msg__VescState__Sequence__init(vesc_msgs__msg__VescState__Sequence * array, size_t size);

/// Finalize array of msg/VescState messages.
/**
 * It calls
 * vesc_msgs__msg__VescState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
void
vesc_msgs__msg__VescState__Sequence__fini(vesc_msgs__msg__VescState__Sequence * array);

/// Create array of msg/VescState messages.
/**
 * It allocates the memory for the array and calls
 * vesc_msgs__msg__VescState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
vesc_msgs__msg__VescState__Sequence *
vesc_msgs__msg__VescState__Sequence__create(size_t size);

/// Destroy array of msg/VescState messages.
/**
 * It calls
 * vesc_msgs__msg__VescState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
void
vesc_msgs__msg__VescState__Sequence__destroy(vesc_msgs__msg__VescState__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // VESC_MSGS__MSG__DETAIL__VESC_STATE__FUNCTIONS_H_
