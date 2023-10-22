// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from interface:action/Cdcr.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__ACTION__DETAIL__CDCR__FUNCTIONS_H_
#define INTERFACE__ACTION__DETAIL__CDCR__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "interface/msg/rosidl_generator_c__visibility_control.h"

#include "interface/action/detail/cdcr__struct.h"

/// Initialize action/Cdcr message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * interface__action__Cdcr_Goal
 * )) before or use
 * interface__action__Cdcr_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_Goal__init(interface__action__Cdcr_Goal * msg);

/// Finalize action/Cdcr message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_Goal__fini(interface__action__Cdcr_Goal * msg);

/// Create action/Cdcr message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * interface__action__Cdcr_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
interface__action__Cdcr_Goal *
interface__action__Cdcr_Goal__create();

/// Destroy action/Cdcr message.
/**
 * It calls
 * interface__action__Cdcr_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_Goal__destroy(interface__action__Cdcr_Goal * msg);

/// Check for action/Cdcr message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_Goal__are_equal(const interface__action__Cdcr_Goal * lhs, const interface__action__Cdcr_Goal * rhs);

/// Copy a action/Cdcr message.
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
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_Goal__copy(
  const interface__action__Cdcr_Goal * input,
  interface__action__Cdcr_Goal * output);

/// Initialize array of action/Cdcr messages.
/**
 * It allocates the memory for the number of elements and calls
 * interface__action__Cdcr_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_Goal__Sequence__init(interface__action__Cdcr_Goal__Sequence * array, size_t size);

/// Finalize array of action/Cdcr messages.
/**
 * It calls
 * interface__action__Cdcr_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_Goal__Sequence__fini(interface__action__Cdcr_Goal__Sequence * array);

/// Create array of action/Cdcr messages.
/**
 * It allocates the memory for the array and calls
 * interface__action__Cdcr_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
interface__action__Cdcr_Goal__Sequence *
interface__action__Cdcr_Goal__Sequence__create(size_t size);

/// Destroy array of action/Cdcr messages.
/**
 * It calls
 * interface__action__Cdcr_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_Goal__Sequence__destroy(interface__action__Cdcr_Goal__Sequence * array);

/// Check for action/Cdcr message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_Goal__Sequence__are_equal(const interface__action__Cdcr_Goal__Sequence * lhs, const interface__action__Cdcr_Goal__Sequence * rhs);

/// Copy an array of action/Cdcr messages.
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
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_Goal__Sequence__copy(
  const interface__action__Cdcr_Goal__Sequence * input,
  interface__action__Cdcr_Goal__Sequence * output);

/// Initialize action/Cdcr message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * interface__action__Cdcr_Result
 * )) before or use
 * interface__action__Cdcr_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_Result__init(interface__action__Cdcr_Result * msg);

/// Finalize action/Cdcr message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_Result__fini(interface__action__Cdcr_Result * msg);

/// Create action/Cdcr message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * interface__action__Cdcr_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
interface__action__Cdcr_Result *
interface__action__Cdcr_Result__create();

/// Destroy action/Cdcr message.
/**
 * It calls
 * interface__action__Cdcr_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_Result__destroy(interface__action__Cdcr_Result * msg);

/// Check for action/Cdcr message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_Result__are_equal(const interface__action__Cdcr_Result * lhs, const interface__action__Cdcr_Result * rhs);

/// Copy a action/Cdcr message.
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
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_Result__copy(
  const interface__action__Cdcr_Result * input,
  interface__action__Cdcr_Result * output);

/// Initialize array of action/Cdcr messages.
/**
 * It allocates the memory for the number of elements and calls
 * interface__action__Cdcr_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_Result__Sequence__init(interface__action__Cdcr_Result__Sequence * array, size_t size);

/// Finalize array of action/Cdcr messages.
/**
 * It calls
 * interface__action__Cdcr_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_Result__Sequence__fini(interface__action__Cdcr_Result__Sequence * array);

/// Create array of action/Cdcr messages.
/**
 * It allocates the memory for the array and calls
 * interface__action__Cdcr_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
interface__action__Cdcr_Result__Sequence *
interface__action__Cdcr_Result__Sequence__create(size_t size);

/// Destroy array of action/Cdcr messages.
/**
 * It calls
 * interface__action__Cdcr_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_Result__Sequence__destroy(interface__action__Cdcr_Result__Sequence * array);

/// Check for action/Cdcr message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_Result__Sequence__are_equal(const interface__action__Cdcr_Result__Sequence * lhs, const interface__action__Cdcr_Result__Sequence * rhs);

/// Copy an array of action/Cdcr messages.
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
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_Result__Sequence__copy(
  const interface__action__Cdcr_Result__Sequence * input,
  interface__action__Cdcr_Result__Sequence * output);

/// Initialize action/Cdcr message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * interface__action__Cdcr_Feedback
 * )) before or use
 * interface__action__Cdcr_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_Feedback__init(interface__action__Cdcr_Feedback * msg);

/// Finalize action/Cdcr message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_Feedback__fini(interface__action__Cdcr_Feedback * msg);

/// Create action/Cdcr message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * interface__action__Cdcr_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
interface__action__Cdcr_Feedback *
interface__action__Cdcr_Feedback__create();

/// Destroy action/Cdcr message.
/**
 * It calls
 * interface__action__Cdcr_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_Feedback__destroy(interface__action__Cdcr_Feedback * msg);

/// Check for action/Cdcr message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_Feedback__are_equal(const interface__action__Cdcr_Feedback * lhs, const interface__action__Cdcr_Feedback * rhs);

/// Copy a action/Cdcr message.
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
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_Feedback__copy(
  const interface__action__Cdcr_Feedback * input,
  interface__action__Cdcr_Feedback * output);

/// Initialize array of action/Cdcr messages.
/**
 * It allocates the memory for the number of elements and calls
 * interface__action__Cdcr_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_Feedback__Sequence__init(interface__action__Cdcr_Feedback__Sequence * array, size_t size);

/// Finalize array of action/Cdcr messages.
/**
 * It calls
 * interface__action__Cdcr_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_Feedback__Sequence__fini(interface__action__Cdcr_Feedback__Sequence * array);

/// Create array of action/Cdcr messages.
/**
 * It allocates the memory for the array and calls
 * interface__action__Cdcr_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
interface__action__Cdcr_Feedback__Sequence *
interface__action__Cdcr_Feedback__Sequence__create(size_t size);

/// Destroy array of action/Cdcr messages.
/**
 * It calls
 * interface__action__Cdcr_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_Feedback__Sequence__destroy(interface__action__Cdcr_Feedback__Sequence * array);

/// Check for action/Cdcr message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_Feedback__Sequence__are_equal(const interface__action__Cdcr_Feedback__Sequence * lhs, const interface__action__Cdcr_Feedback__Sequence * rhs);

/// Copy an array of action/Cdcr messages.
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
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_Feedback__Sequence__copy(
  const interface__action__Cdcr_Feedback__Sequence * input,
  interface__action__Cdcr_Feedback__Sequence * output);

/// Initialize action/Cdcr message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * interface__action__Cdcr_SendGoal_Request
 * )) before or use
 * interface__action__Cdcr_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_SendGoal_Request__init(interface__action__Cdcr_SendGoal_Request * msg);

/// Finalize action/Cdcr message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_SendGoal_Request__fini(interface__action__Cdcr_SendGoal_Request * msg);

/// Create action/Cdcr message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * interface__action__Cdcr_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
interface__action__Cdcr_SendGoal_Request *
interface__action__Cdcr_SendGoal_Request__create();

/// Destroy action/Cdcr message.
/**
 * It calls
 * interface__action__Cdcr_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_SendGoal_Request__destroy(interface__action__Cdcr_SendGoal_Request * msg);

/// Check for action/Cdcr message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_SendGoal_Request__are_equal(const interface__action__Cdcr_SendGoal_Request * lhs, const interface__action__Cdcr_SendGoal_Request * rhs);

/// Copy a action/Cdcr message.
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
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_SendGoal_Request__copy(
  const interface__action__Cdcr_SendGoal_Request * input,
  interface__action__Cdcr_SendGoal_Request * output);

/// Initialize array of action/Cdcr messages.
/**
 * It allocates the memory for the number of elements and calls
 * interface__action__Cdcr_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_SendGoal_Request__Sequence__init(interface__action__Cdcr_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/Cdcr messages.
/**
 * It calls
 * interface__action__Cdcr_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_SendGoal_Request__Sequence__fini(interface__action__Cdcr_SendGoal_Request__Sequence * array);

/// Create array of action/Cdcr messages.
/**
 * It allocates the memory for the array and calls
 * interface__action__Cdcr_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
interface__action__Cdcr_SendGoal_Request__Sequence *
interface__action__Cdcr_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/Cdcr messages.
/**
 * It calls
 * interface__action__Cdcr_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_SendGoal_Request__Sequence__destroy(interface__action__Cdcr_SendGoal_Request__Sequence * array);

/// Check for action/Cdcr message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_SendGoal_Request__Sequence__are_equal(const interface__action__Cdcr_SendGoal_Request__Sequence * lhs, const interface__action__Cdcr_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/Cdcr messages.
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
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_SendGoal_Request__Sequence__copy(
  const interface__action__Cdcr_SendGoal_Request__Sequence * input,
  interface__action__Cdcr_SendGoal_Request__Sequence * output);

/// Initialize action/Cdcr message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * interface__action__Cdcr_SendGoal_Response
 * )) before or use
 * interface__action__Cdcr_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_SendGoal_Response__init(interface__action__Cdcr_SendGoal_Response * msg);

/// Finalize action/Cdcr message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_SendGoal_Response__fini(interface__action__Cdcr_SendGoal_Response * msg);

/// Create action/Cdcr message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * interface__action__Cdcr_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
interface__action__Cdcr_SendGoal_Response *
interface__action__Cdcr_SendGoal_Response__create();

/// Destroy action/Cdcr message.
/**
 * It calls
 * interface__action__Cdcr_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_SendGoal_Response__destroy(interface__action__Cdcr_SendGoal_Response * msg);

/// Check for action/Cdcr message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_SendGoal_Response__are_equal(const interface__action__Cdcr_SendGoal_Response * lhs, const interface__action__Cdcr_SendGoal_Response * rhs);

/// Copy a action/Cdcr message.
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
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_SendGoal_Response__copy(
  const interface__action__Cdcr_SendGoal_Response * input,
  interface__action__Cdcr_SendGoal_Response * output);

/// Initialize array of action/Cdcr messages.
/**
 * It allocates the memory for the number of elements and calls
 * interface__action__Cdcr_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_SendGoal_Response__Sequence__init(interface__action__Cdcr_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/Cdcr messages.
/**
 * It calls
 * interface__action__Cdcr_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_SendGoal_Response__Sequence__fini(interface__action__Cdcr_SendGoal_Response__Sequence * array);

/// Create array of action/Cdcr messages.
/**
 * It allocates the memory for the array and calls
 * interface__action__Cdcr_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
interface__action__Cdcr_SendGoal_Response__Sequence *
interface__action__Cdcr_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/Cdcr messages.
/**
 * It calls
 * interface__action__Cdcr_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_SendGoal_Response__Sequence__destroy(interface__action__Cdcr_SendGoal_Response__Sequence * array);

/// Check for action/Cdcr message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_SendGoal_Response__Sequence__are_equal(const interface__action__Cdcr_SendGoal_Response__Sequence * lhs, const interface__action__Cdcr_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/Cdcr messages.
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
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_SendGoal_Response__Sequence__copy(
  const interface__action__Cdcr_SendGoal_Response__Sequence * input,
  interface__action__Cdcr_SendGoal_Response__Sequence * output);

/// Initialize action/Cdcr message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * interface__action__Cdcr_GetResult_Request
 * )) before or use
 * interface__action__Cdcr_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_GetResult_Request__init(interface__action__Cdcr_GetResult_Request * msg);

/// Finalize action/Cdcr message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_GetResult_Request__fini(interface__action__Cdcr_GetResult_Request * msg);

/// Create action/Cdcr message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * interface__action__Cdcr_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
interface__action__Cdcr_GetResult_Request *
interface__action__Cdcr_GetResult_Request__create();

/// Destroy action/Cdcr message.
/**
 * It calls
 * interface__action__Cdcr_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_GetResult_Request__destroy(interface__action__Cdcr_GetResult_Request * msg);

/// Check for action/Cdcr message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_GetResult_Request__are_equal(const interface__action__Cdcr_GetResult_Request * lhs, const interface__action__Cdcr_GetResult_Request * rhs);

/// Copy a action/Cdcr message.
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
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_GetResult_Request__copy(
  const interface__action__Cdcr_GetResult_Request * input,
  interface__action__Cdcr_GetResult_Request * output);

/// Initialize array of action/Cdcr messages.
/**
 * It allocates the memory for the number of elements and calls
 * interface__action__Cdcr_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_GetResult_Request__Sequence__init(interface__action__Cdcr_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/Cdcr messages.
/**
 * It calls
 * interface__action__Cdcr_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_GetResult_Request__Sequence__fini(interface__action__Cdcr_GetResult_Request__Sequence * array);

/// Create array of action/Cdcr messages.
/**
 * It allocates the memory for the array and calls
 * interface__action__Cdcr_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
interface__action__Cdcr_GetResult_Request__Sequence *
interface__action__Cdcr_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/Cdcr messages.
/**
 * It calls
 * interface__action__Cdcr_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_GetResult_Request__Sequence__destroy(interface__action__Cdcr_GetResult_Request__Sequence * array);

/// Check for action/Cdcr message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_GetResult_Request__Sequence__are_equal(const interface__action__Cdcr_GetResult_Request__Sequence * lhs, const interface__action__Cdcr_GetResult_Request__Sequence * rhs);

/// Copy an array of action/Cdcr messages.
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
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_GetResult_Request__Sequence__copy(
  const interface__action__Cdcr_GetResult_Request__Sequence * input,
  interface__action__Cdcr_GetResult_Request__Sequence * output);

/// Initialize action/Cdcr message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * interface__action__Cdcr_GetResult_Response
 * )) before or use
 * interface__action__Cdcr_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_GetResult_Response__init(interface__action__Cdcr_GetResult_Response * msg);

/// Finalize action/Cdcr message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_GetResult_Response__fini(interface__action__Cdcr_GetResult_Response * msg);

/// Create action/Cdcr message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * interface__action__Cdcr_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
interface__action__Cdcr_GetResult_Response *
interface__action__Cdcr_GetResult_Response__create();

/// Destroy action/Cdcr message.
/**
 * It calls
 * interface__action__Cdcr_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_GetResult_Response__destroy(interface__action__Cdcr_GetResult_Response * msg);

/// Check for action/Cdcr message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_GetResult_Response__are_equal(const interface__action__Cdcr_GetResult_Response * lhs, const interface__action__Cdcr_GetResult_Response * rhs);

/// Copy a action/Cdcr message.
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
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_GetResult_Response__copy(
  const interface__action__Cdcr_GetResult_Response * input,
  interface__action__Cdcr_GetResult_Response * output);

/// Initialize array of action/Cdcr messages.
/**
 * It allocates the memory for the number of elements and calls
 * interface__action__Cdcr_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_GetResult_Response__Sequence__init(interface__action__Cdcr_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/Cdcr messages.
/**
 * It calls
 * interface__action__Cdcr_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_GetResult_Response__Sequence__fini(interface__action__Cdcr_GetResult_Response__Sequence * array);

/// Create array of action/Cdcr messages.
/**
 * It allocates the memory for the array and calls
 * interface__action__Cdcr_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
interface__action__Cdcr_GetResult_Response__Sequence *
interface__action__Cdcr_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/Cdcr messages.
/**
 * It calls
 * interface__action__Cdcr_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_GetResult_Response__Sequence__destroy(interface__action__Cdcr_GetResult_Response__Sequence * array);

/// Check for action/Cdcr message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_GetResult_Response__Sequence__are_equal(const interface__action__Cdcr_GetResult_Response__Sequence * lhs, const interface__action__Cdcr_GetResult_Response__Sequence * rhs);

/// Copy an array of action/Cdcr messages.
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
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_GetResult_Response__Sequence__copy(
  const interface__action__Cdcr_GetResult_Response__Sequence * input,
  interface__action__Cdcr_GetResult_Response__Sequence * output);

/// Initialize action/Cdcr message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * interface__action__Cdcr_FeedbackMessage
 * )) before or use
 * interface__action__Cdcr_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_FeedbackMessage__init(interface__action__Cdcr_FeedbackMessage * msg);

/// Finalize action/Cdcr message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_FeedbackMessage__fini(interface__action__Cdcr_FeedbackMessage * msg);

/// Create action/Cdcr message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * interface__action__Cdcr_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
interface__action__Cdcr_FeedbackMessage *
interface__action__Cdcr_FeedbackMessage__create();

/// Destroy action/Cdcr message.
/**
 * It calls
 * interface__action__Cdcr_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_FeedbackMessage__destroy(interface__action__Cdcr_FeedbackMessage * msg);

/// Check for action/Cdcr message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_FeedbackMessage__are_equal(const interface__action__Cdcr_FeedbackMessage * lhs, const interface__action__Cdcr_FeedbackMessage * rhs);

/// Copy a action/Cdcr message.
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
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_FeedbackMessage__copy(
  const interface__action__Cdcr_FeedbackMessage * input,
  interface__action__Cdcr_FeedbackMessage * output);

/// Initialize array of action/Cdcr messages.
/**
 * It allocates the memory for the number of elements and calls
 * interface__action__Cdcr_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_FeedbackMessage__Sequence__init(interface__action__Cdcr_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/Cdcr messages.
/**
 * It calls
 * interface__action__Cdcr_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_FeedbackMessage__Sequence__fini(interface__action__Cdcr_FeedbackMessage__Sequence * array);

/// Create array of action/Cdcr messages.
/**
 * It allocates the memory for the array and calls
 * interface__action__Cdcr_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
interface__action__Cdcr_FeedbackMessage__Sequence *
interface__action__Cdcr_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/Cdcr messages.
/**
 * It calls
 * interface__action__Cdcr_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
void
interface__action__Cdcr_FeedbackMessage__Sequence__destroy(interface__action__Cdcr_FeedbackMessage__Sequence * array);

/// Check for action/Cdcr message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_FeedbackMessage__Sequence__are_equal(const interface__action__Cdcr_FeedbackMessage__Sequence * lhs, const interface__action__Cdcr_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/Cdcr messages.
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
ROSIDL_GENERATOR_C_PUBLIC_interface
bool
interface__action__Cdcr_FeedbackMessage__Sequence__copy(
  const interface__action__Cdcr_FeedbackMessage__Sequence * input,
  interface__action__Cdcr_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE__ACTION__DETAIL__CDCR__FUNCTIONS_H_
