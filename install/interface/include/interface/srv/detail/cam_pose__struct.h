// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface:srv/CamPose.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__CAM_POSE__STRUCT_H_
#define INTERFACE__SRV__DETAIL__CAM_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/CamPose in the package interface.
typedef struct interface__srv__CamPose_Request
{
  uint8_t structure_needs_at_least_one_member;
} interface__srv__CamPose_Request;

// Struct for a sequence of interface__srv__CamPose_Request.
typedef struct interface__srv__CamPose_Request__Sequence
{
  interface__srv__CamPose_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__srv__CamPose_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'cam_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

// Struct defined in srv/CamPose in the package interface.
typedef struct interface__srv__CamPose_Response
{
  geometry_msgs__msg__Pose cam_pose;
} interface__srv__CamPose_Response;

// Struct for a sequence of interface__srv__CamPose_Response.
typedef struct interface__srv__CamPose_Response__Sequence
{
  interface__srv__CamPose_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__srv__CamPose_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE__SRV__DETAIL__CAM_POSE__STRUCT_H_
