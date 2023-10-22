// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface:action/Cdcr.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__ACTION__DETAIL__CDCR__STRUCT_H_
#define INTERFACE__ACTION__DETAIL__CDCR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'goal_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

// Struct defined in action/Cdcr in the package interface.
typedef struct interface__action__Cdcr_Goal
{
  geometry_msgs__msg__Pose goal_pose;
} interface__action__Cdcr_Goal;

// Struct for a sequence of interface__action__Cdcr_Goal.
typedef struct interface__action__Cdcr_Goal__Sequence
{
  interface__action__Cdcr_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__action__Cdcr_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose__struct.h"

// Struct defined in action/Cdcr in the package interface.
typedef struct interface__action__Cdcr_Result
{
  geometry_msgs__msg__Pose result_pose;
} interface__action__Cdcr_Result;

// Struct for a sequence of interface__action__Cdcr_Result.
typedef struct interface__action__Cdcr_Result__Sequence
{
  interface__action__Cdcr_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__action__Cdcr_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'feedback_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose__struct.h"

// Struct defined in action/Cdcr in the package interface.
typedef struct interface__action__Cdcr_Feedback
{
  geometry_msgs__msg__Pose feedback_pose;
} interface__action__Cdcr_Feedback;

// Struct for a sequence of interface__action__Cdcr_Feedback.
typedef struct interface__action__Cdcr_Feedback__Sequence
{
  interface__action__Cdcr_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__action__Cdcr_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "interface/action/detail/cdcr__struct.h"

// Struct defined in action/Cdcr in the package interface.
typedef struct interface__action__Cdcr_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  interface__action__Cdcr_Goal goal;
} interface__action__Cdcr_SendGoal_Request;

// Struct for a sequence of interface__action__Cdcr_SendGoal_Request.
typedef struct interface__action__Cdcr_SendGoal_Request__Sequence
{
  interface__action__Cdcr_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__action__Cdcr_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

// Struct defined in action/Cdcr in the package interface.
typedef struct interface__action__Cdcr_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} interface__action__Cdcr_SendGoal_Response;

// Struct for a sequence of interface__action__Cdcr_SendGoal_Response.
typedef struct interface__action__Cdcr_SendGoal_Response__Sequence
{
  interface__action__Cdcr_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__action__Cdcr_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

// Struct defined in action/Cdcr in the package interface.
typedef struct interface__action__Cdcr_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} interface__action__Cdcr_GetResult_Request;

// Struct for a sequence of interface__action__Cdcr_GetResult_Request.
typedef struct interface__action__Cdcr_GetResult_Request__Sequence
{
  interface__action__Cdcr_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__action__Cdcr_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "interface/action/detail/cdcr__struct.h"

// Struct defined in action/Cdcr in the package interface.
typedef struct interface__action__Cdcr_GetResult_Response
{
  int8_t status;
  interface__action__Cdcr_Result result;
} interface__action__Cdcr_GetResult_Response;

// Struct for a sequence of interface__action__Cdcr_GetResult_Response.
typedef struct interface__action__Cdcr_GetResult_Response__Sequence
{
  interface__action__Cdcr_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__action__Cdcr_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "interface/action/detail/cdcr__struct.h"

// Struct defined in action/Cdcr in the package interface.
typedef struct interface__action__Cdcr_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  interface__action__Cdcr_Feedback feedback;
} interface__action__Cdcr_FeedbackMessage;

// Struct for a sequence of interface__action__Cdcr_FeedbackMessage.
typedef struct interface__action__Cdcr_FeedbackMessage__Sequence
{
  interface__action__Cdcr_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__action__Cdcr_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE__ACTION__DETAIL__CDCR__STRUCT_H_
