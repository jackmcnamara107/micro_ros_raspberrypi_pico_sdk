// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from example_interfaces:msg/MultiArrayLayout.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "example_interfaces/msg/multi_array_layout.h"


#ifndef EXAMPLE_INTERFACES__MSG__DETAIL__MULTI_ARRAY_LAYOUT__FUNCTIONS_H_
#define EXAMPLE_INTERFACES__MSG__DETAIL__MULTI_ARRAY_LAYOUT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "example_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "example_interfaces/msg/detail/multi_array_layout__struct.h"

/// Initialize msg/MultiArrayLayout message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * example_interfaces__msg__MultiArrayLayout
 * )) before or use
 * example_interfaces__msg__MultiArrayLayout__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_example_interfaces
bool
example_interfaces__msg__MultiArrayLayout__init(example_interfaces__msg__MultiArrayLayout * msg);

/// Finalize msg/MultiArrayLayout message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_example_interfaces
void
example_interfaces__msg__MultiArrayLayout__fini(example_interfaces__msg__MultiArrayLayout * msg);

/// Create msg/MultiArrayLayout message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * example_interfaces__msg__MultiArrayLayout__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_example_interfaces
example_interfaces__msg__MultiArrayLayout *
example_interfaces__msg__MultiArrayLayout__create();

/// Destroy msg/MultiArrayLayout message.
/**
 * It calls
 * example_interfaces__msg__MultiArrayLayout__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_example_interfaces
void
example_interfaces__msg__MultiArrayLayout__destroy(example_interfaces__msg__MultiArrayLayout * msg);

/// Check for msg/MultiArrayLayout message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_example_interfaces
bool
example_interfaces__msg__MultiArrayLayout__are_equal(const example_interfaces__msg__MultiArrayLayout * lhs, const example_interfaces__msg__MultiArrayLayout * rhs);

/// Copy a msg/MultiArrayLayout message.
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
ROSIDL_GENERATOR_C_PUBLIC_example_interfaces
bool
example_interfaces__msg__MultiArrayLayout__copy(
  const example_interfaces__msg__MultiArrayLayout * input,
  example_interfaces__msg__MultiArrayLayout * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_example_interfaces
const rosidl_type_hash_t *
example_interfaces__msg__MultiArrayLayout__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_example_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
example_interfaces__msg__MultiArrayLayout__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_example_interfaces
const rosidl_runtime_c__type_description__TypeSource *
example_interfaces__msg__MultiArrayLayout__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_example_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
example_interfaces__msg__MultiArrayLayout__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/MultiArrayLayout messages.
/**
 * It allocates the memory for the number of elements and calls
 * example_interfaces__msg__MultiArrayLayout__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_example_interfaces
bool
example_interfaces__msg__MultiArrayLayout__Sequence__init(example_interfaces__msg__MultiArrayLayout__Sequence * array, size_t size);

/// Finalize array of msg/MultiArrayLayout messages.
/**
 * It calls
 * example_interfaces__msg__MultiArrayLayout__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_example_interfaces
void
example_interfaces__msg__MultiArrayLayout__Sequence__fini(example_interfaces__msg__MultiArrayLayout__Sequence * array);

/// Create array of msg/MultiArrayLayout messages.
/**
 * It allocates the memory for the array and calls
 * example_interfaces__msg__MultiArrayLayout__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_example_interfaces
example_interfaces__msg__MultiArrayLayout__Sequence *
example_interfaces__msg__MultiArrayLayout__Sequence__create(size_t size);

/// Destroy array of msg/MultiArrayLayout messages.
/**
 * It calls
 * example_interfaces__msg__MultiArrayLayout__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_example_interfaces
void
example_interfaces__msg__MultiArrayLayout__Sequence__destroy(example_interfaces__msg__MultiArrayLayout__Sequence * array);

/// Check for msg/MultiArrayLayout message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_example_interfaces
bool
example_interfaces__msg__MultiArrayLayout__Sequence__are_equal(const example_interfaces__msg__MultiArrayLayout__Sequence * lhs, const example_interfaces__msg__MultiArrayLayout__Sequence * rhs);

/// Copy an array of msg/MultiArrayLayout messages.
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
ROSIDL_GENERATOR_C_PUBLIC_example_interfaces
bool
example_interfaces__msg__MultiArrayLayout__Sequence__copy(
  const example_interfaces__msg__MultiArrayLayout__Sequence * input,
  example_interfaces__msg__MultiArrayLayout__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // EXAMPLE_INTERFACES__MSG__DETAIL__MULTI_ARRAY_LAYOUT__FUNCTIONS_H_
