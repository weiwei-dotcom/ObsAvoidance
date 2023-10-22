// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface:srv/SlamInitialized.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__SLAM_INITIALIZED__STRUCT_H_
#define INTERFACE__SRV__DETAIL__SLAM_INITIALIZED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/SlamInitialized in the package interface.
typedef struct interface__srv__SlamInitialized_Request
{
  uint8_t structure_needs_at_least_one_member;
} interface__srv__SlamInitialized_Request;

// Struct for a sequence of interface__srv__SlamInitialized_Request.
typedef struct interface__srv__SlamInitialized_Request__Sequence
{
  interface__srv__SlamInitialized_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__srv__SlamInitialized_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/SlamInitialized in the package interface.
typedef struct interface__srv__SlamInitialized_Response
{
  bool flag_slam_initialized;
} interface__srv__SlamInitialized_Response;

// Struct for a sequence of interface__srv__SlamInitialized_Response.
typedef struct interface__srv__SlamInitialized_Response__Sequence
{
  interface__srv__SlamInitialized_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__srv__SlamInitialized_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE__SRV__DETAIL__SLAM_INITIALIZED__STRUCT_H_
