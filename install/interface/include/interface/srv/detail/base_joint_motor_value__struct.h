// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface:srv/BaseJointMotorValue.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__BASE_JOINT_MOTOR_VALUE__STRUCT_H_
#define INTERFACE__SRV__DETAIL__BASE_JOINT_MOTOR_VALUE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/BaseJointMotorValue in the package interface.
typedef struct interface__srv__BaseJointMotorValue_Request
{
  uint8_t structure_needs_at_least_one_member;
} interface__srv__BaseJointMotorValue_Request;

// Struct for a sequence of interface__srv__BaseJointMotorValue_Request.
typedef struct interface__srv__BaseJointMotorValue_Request__Sequence
{
  interface__srv__BaseJointMotorValue_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__srv__BaseJointMotorValue_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'cable_expend_value'
#include "geometry_msgs/msg/detail/point__struct.h"

// Struct defined in srv/BaseJointMotorValue in the package interface.
typedef struct interface__srv__BaseJointMotorValue_Response
{
  geometry_msgs__msg__Point__Sequence cable_expend_value;
  double base_advance_value;
  bool flag_base_out_range;
  bool flag_arrived_target;
} interface__srv__BaseJointMotorValue_Response;

// Struct for a sequence of interface__srv__BaseJointMotorValue_Response.
typedef struct interface__srv__BaseJointMotorValue_Response__Sequence
{
  interface__srv__BaseJointMotorValue_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__srv__BaseJointMotorValue_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE__SRV__DETAIL__BASE_JOINT_MOTOR_VALUE__STRUCT_H_
