// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interface:action/Move.idl
// generated code does not contain a copyright notice
#include "interface/action/detail/move__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `goal_pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
interface__action__Move_Goal__init(interface__action__Move_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // goal_pose
  if (!geometry_msgs__msg__Pose__init(&msg->goal_pose)) {
    interface__action__Move_Goal__fini(msg);
    return false;
  }
  return true;
}

void
interface__action__Move_Goal__fini(interface__action__Move_Goal * msg)
{
  if (!msg) {
    return;
  }
  // goal_pose
  geometry_msgs__msg__Pose__fini(&msg->goal_pose);
}

bool
interface__action__Move_Goal__are_equal(const interface__action__Move_Goal * lhs, const interface__action__Move_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->goal_pose), &(rhs->goal_pose)))
  {
    return false;
  }
  return true;
}

bool
interface__action__Move_Goal__copy(
  const interface__action__Move_Goal * input,
  interface__action__Move_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->goal_pose), &(output->goal_pose)))
  {
    return false;
  }
  return true;
}

interface__action__Move_Goal *
interface__action__Move_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_Goal * msg = (interface__action__Move_Goal *)allocator.allocate(sizeof(interface__action__Move_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface__action__Move_Goal));
  bool success = interface__action__Move_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface__action__Move_Goal__destroy(interface__action__Move_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface__action__Move_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface__action__Move_Goal__Sequence__init(interface__action__Move_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_Goal * data = NULL;

  if (size) {
    data = (interface__action__Move_Goal *)allocator.zero_allocate(size, sizeof(interface__action__Move_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface__action__Move_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface__action__Move_Goal__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
interface__action__Move_Goal__Sequence__fini(interface__action__Move_Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      interface__action__Move_Goal__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

interface__action__Move_Goal__Sequence *
interface__action__Move_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_Goal__Sequence * array = (interface__action__Move_Goal__Sequence *)allocator.allocate(sizeof(interface__action__Move_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface__action__Move_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface__action__Move_Goal__Sequence__destroy(interface__action__Move_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface__action__Move_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface__action__Move_Goal__Sequence__are_equal(const interface__action__Move_Goal__Sequence * lhs, const interface__action__Move_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface__action__Move_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface__action__Move_Goal__Sequence__copy(
  const interface__action__Move_Goal__Sequence * input,
  interface__action__Move_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface__action__Move_Goal);
    interface__action__Move_Goal * data =
      (interface__action__Move_Goal *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface__action__Move_Goal__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          interface__action__Move_Goal__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface__action__Move_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result_pose`
// already included above
// #include "geometry_msgs/msg/detail/pose__functions.h"

bool
interface__action__Move_Result__init(interface__action__Move_Result * msg)
{
  if (!msg) {
    return false;
  }
  // result_pose
  if (!geometry_msgs__msg__Pose__init(&msg->result_pose)) {
    interface__action__Move_Result__fini(msg);
    return false;
  }
  return true;
}

void
interface__action__Move_Result__fini(interface__action__Move_Result * msg)
{
  if (!msg) {
    return;
  }
  // result_pose
  geometry_msgs__msg__Pose__fini(&msg->result_pose);
}

bool
interface__action__Move_Result__are_equal(const interface__action__Move_Result * lhs, const interface__action__Move_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // result_pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->result_pose), &(rhs->result_pose)))
  {
    return false;
  }
  return true;
}

bool
interface__action__Move_Result__copy(
  const interface__action__Move_Result * input,
  interface__action__Move_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // result_pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->result_pose), &(output->result_pose)))
  {
    return false;
  }
  return true;
}

interface__action__Move_Result *
interface__action__Move_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_Result * msg = (interface__action__Move_Result *)allocator.allocate(sizeof(interface__action__Move_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface__action__Move_Result));
  bool success = interface__action__Move_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface__action__Move_Result__destroy(interface__action__Move_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface__action__Move_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface__action__Move_Result__Sequence__init(interface__action__Move_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_Result * data = NULL;

  if (size) {
    data = (interface__action__Move_Result *)allocator.zero_allocate(size, sizeof(interface__action__Move_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface__action__Move_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface__action__Move_Result__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
interface__action__Move_Result__Sequence__fini(interface__action__Move_Result__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      interface__action__Move_Result__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

interface__action__Move_Result__Sequence *
interface__action__Move_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_Result__Sequence * array = (interface__action__Move_Result__Sequence *)allocator.allocate(sizeof(interface__action__Move_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface__action__Move_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface__action__Move_Result__Sequence__destroy(interface__action__Move_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface__action__Move_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface__action__Move_Result__Sequence__are_equal(const interface__action__Move_Result__Sequence * lhs, const interface__action__Move_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface__action__Move_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface__action__Move_Result__Sequence__copy(
  const interface__action__Move_Result__Sequence * input,
  interface__action__Move_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface__action__Move_Result);
    interface__action__Move_Result * data =
      (interface__action__Move_Result *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface__action__Move_Result__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          interface__action__Move_Result__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface__action__Move_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `feedback_pose`
// already included above
// #include "geometry_msgs/msg/detail/pose__functions.h"

bool
interface__action__Move_Feedback__init(interface__action__Move_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // feedback_pose
  if (!geometry_msgs__msg__Pose__init(&msg->feedback_pose)) {
    interface__action__Move_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
interface__action__Move_Feedback__fini(interface__action__Move_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // feedback_pose
  geometry_msgs__msg__Pose__fini(&msg->feedback_pose);
}

bool
interface__action__Move_Feedback__are_equal(const interface__action__Move_Feedback * lhs, const interface__action__Move_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // feedback_pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->feedback_pose), &(rhs->feedback_pose)))
  {
    return false;
  }
  return true;
}

bool
interface__action__Move_Feedback__copy(
  const interface__action__Move_Feedback * input,
  interface__action__Move_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // feedback_pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->feedback_pose), &(output->feedback_pose)))
  {
    return false;
  }
  return true;
}

interface__action__Move_Feedback *
interface__action__Move_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_Feedback * msg = (interface__action__Move_Feedback *)allocator.allocate(sizeof(interface__action__Move_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface__action__Move_Feedback));
  bool success = interface__action__Move_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface__action__Move_Feedback__destroy(interface__action__Move_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface__action__Move_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface__action__Move_Feedback__Sequence__init(interface__action__Move_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_Feedback * data = NULL;

  if (size) {
    data = (interface__action__Move_Feedback *)allocator.zero_allocate(size, sizeof(interface__action__Move_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface__action__Move_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface__action__Move_Feedback__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
interface__action__Move_Feedback__Sequence__fini(interface__action__Move_Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      interface__action__Move_Feedback__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

interface__action__Move_Feedback__Sequence *
interface__action__Move_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_Feedback__Sequence * array = (interface__action__Move_Feedback__Sequence *)allocator.allocate(sizeof(interface__action__Move_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface__action__Move_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface__action__Move_Feedback__Sequence__destroy(interface__action__Move_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface__action__Move_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface__action__Move_Feedback__Sequence__are_equal(const interface__action__Move_Feedback__Sequence * lhs, const interface__action__Move_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface__action__Move_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface__action__Move_Feedback__Sequence__copy(
  const interface__action__Move_Feedback__Sequence * input,
  interface__action__Move_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface__action__Move_Feedback);
    interface__action__Move_Feedback * data =
      (interface__action__Move_Feedback *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface__action__Move_Feedback__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          interface__action__Move_Feedback__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface__action__Move_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "interface/action/detail/move__functions.h"

bool
interface__action__Move_SendGoal_Request__init(interface__action__Move_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    interface__action__Move_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!interface__action__Move_Goal__init(&msg->goal)) {
    interface__action__Move_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
interface__action__Move_SendGoal_Request__fini(interface__action__Move_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  interface__action__Move_Goal__fini(&msg->goal);
}

bool
interface__action__Move_SendGoal_Request__are_equal(const interface__action__Move_SendGoal_Request * lhs, const interface__action__Move_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!interface__action__Move_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
interface__action__Move_SendGoal_Request__copy(
  const interface__action__Move_SendGoal_Request * input,
  interface__action__Move_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!interface__action__Move_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

interface__action__Move_SendGoal_Request *
interface__action__Move_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_SendGoal_Request * msg = (interface__action__Move_SendGoal_Request *)allocator.allocate(sizeof(interface__action__Move_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface__action__Move_SendGoal_Request));
  bool success = interface__action__Move_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface__action__Move_SendGoal_Request__destroy(interface__action__Move_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface__action__Move_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface__action__Move_SendGoal_Request__Sequence__init(interface__action__Move_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_SendGoal_Request * data = NULL;

  if (size) {
    data = (interface__action__Move_SendGoal_Request *)allocator.zero_allocate(size, sizeof(interface__action__Move_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface__action__Move_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface__action__Move_SendGoal_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
interface__action__Move_SendGoal_Request__Sequence__fini(interface__action__Move_SendGoal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      interface__action__Move_SendGoal_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

interface__action__Move_SendGoal_Request__Sequence *
interface__action__Move_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_SendGoal_Request__Sequence * array = (interface__action__Move_SendGoal_Request__Sequence *)allocator.allocate(sizeof(interface__action__Move_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface__action__Move_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface__action__Move_SendGoal_Request__Sequence__destroy(interface__action__Move_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface__action__Move_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface__action__Move_SendGoal_Request__Sequence__are_equal(const interface__action__Move_SendGoal_Request__Sequence * lhs, const interface__action__Move_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface__action__Move_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface__action__Move_SendGoal_Request__Sequence__copy(
  const interface__action__Move_SendGoal_Request__Sequence * input,
  interface__action__Move_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface__action__Move_SendGoal_Request);
    interface__action__Move_SendGoal_Request * data =
      (interface__action__Move_SendGoal_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface__action__Move_SendGoal_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          interface__action__Move_SendGoal_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface__action__Move_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
interface__action__Move_SendGoal_Response__init(interface__action__Move_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    interface__action__Move_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
interface__action__Move_SendGoal_Response__fini(interface__action__Move_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
interface__action__Move_SendGoal_Response__are_equal(const interface__action__Move_SendGoal_Response * lhs, const interface__action__Move_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
interface__action__Move_SendGoal_Response__copy(
  const interface__action__Move_SendGoal_Response * input,
  interface__action__Move_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

interface__action__Move_SendGoal_Response *
interface__action__Move_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_SendGoal_Response * msg = (interface__action__Move_SendGoal_Response *)allocator.allocate(sizeof(interface__action__Move_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface__action__Move_SendGoal_Response));
  bool success = interface__action__Move_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface__action__Move_SendGoal_Response__destroy(interface__action__Move_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface__action__Move_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface__action__Move_SendGoal_Response__Sequence__init(interface__action__Move_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_SendGoal_Response * data = NULL;

  if (size) {
    data = (interface__action__Move_SendGoal_Response *)allocator.zero_allocate(size, sizeof(interface__action__Move_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface__action__Move_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface__action__Move_SendGoal_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
interface__action__Move_SendGoal_Response__Sequence__fini(interface__action__Move_SendGoal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      interface__action__Move_SendGoal_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

interface__action__Move_SendGoal_Response__Sequence *
interface__action__Move_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_SendGoal_Response__Sequence * array = (interface__action__Move_SendGoal_Response__Sequence *)allocator.allocate(sizeof(interface__action__Move_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface__action__Move_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface__action__Move_SendGoal_Response__Sequence__destroy(interface__action__Move_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface__action__Move_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface__action__Move_SendGoal_Response__Sequence__are_equal(const interface__action__Move_SendGoal_Response__Sequence * lhs, const interface__action__Move_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface__action__Move_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface__action__Move_SendGoal_Response__Sequence__copy(
  const interface__action__Move_SendGoal_Response__Sequence * input,
  interface__action__Move_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface__action__Move_SendGoal_Response);
    interface__action__Move_SendGoal_Response * data =
      (interface__action__Move_SendGoal_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface__action__Move_SendGoal_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          interface__action__Move_SendGoal_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface__action__Move_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
interface__action__Move_GetResult_Request__init(interface__action__Move_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    interface__action__Move_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
interface__action__Move_GetResult_Request__fini(interface__action__Move_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
interface__action__Move_GetResult_Request__are_equal(const interface__action__Move_GetResult_Request * lhs, const interface__action__Move_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
interface__action__Move_GetResult_Request__copy(
  const interface__action__Move_GetResult_Request * input,
  interface__action__Move_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

interface__action__Move_GetResult_Request *
interface__action__Move_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_GetResult_Request * msg = (interface__action__Move_GetResult_Request *)allocator.allocate(sizeof(interface__action__Move_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface__action__Move_GetResult_Request));
  bool success = interface__action__Move_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface__action__Move_GetResult_Request__destroy(interface__action__Move_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface__action__Move_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface__action__Move_GetResult_Request__Sequence__init(interface__action__Move_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_GetResult_Request * data = NULL;

  if (size) {
    data = (interface__action__Move_GetResult_Request *)allocator.zero_allocate(size, sizeof(interface__action__Move_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface__action__Move_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface__action__Move_GetResult_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
interface__action__Move_GetResult_Request__Sequence__fini(interface__action__Move_GetResult_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      interface__action__Move_GetResult_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

interface__action__Move_GetResult_Request__Sequence *
interface__action__Move_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_GetResult_Request__Sequence * array = (interface__action__Move_GetResult_Request__Sequence *)allocator.allocate(sizeof(interface__action__Move_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface__action__Move_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface__action__Move_GetResult_Request__Sequence__destroy(interface__action__Move_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface__action__Move_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface__action__Move_GetResult_Request__Sequence__are_equal(const interface__action__Move_GetResult_Request__Sequence * lhs, const interface__action__Move_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface__action__Move_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface__action__Move_GetResult_Request__Sequence__copy(
  const interface__action__Move_GetResult_Request__Sequence * input,
  interface__action__Move_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface__action__Move_GetResult_Request);
    interface__action__Move_GetResult_Request * data =
      (interface__action__Move_GetResult_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface__action__Move_GetResult_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          interface__action__Move_GetResult_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface__action__Move_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "interface/action/detail/move__functions.h"

bool
interface__action__Move_GetResult_Response__init(interface__action__Move_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!interface__action__Move_Result__init(&msg->result)) {
    interface__action__Move_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
interface__action__Move_GetResult_Response__fini(interface__action__Move_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  interface__action__Move_Result__fini(&msg->result);
}

bool
interface__action__Move_GetResult_Response__are_equal(const interface__action__Move_GetResult_Response * lhs, const interface__action__Move_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!interface__action__Move_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
interface__action__Move_GetResult_Response__copy(
  const interface__action__Move_GetResult_Response * input,
  interface__action__Move_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!interface__action__Move_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

interface__action__Move_GetResult_Response *
interface__action__Move_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_GetResult_Response * msg = (interface__action__Move_GetResult_Response *)allocator.allocate(sizeof(interface__action__Move_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface__action__Move_GetResult_Response));
  bool success = interface__action__Move_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface__action__Move_GetResult_Response__destroy(interface__action__Move_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface__action__Move_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface__action__Move_GetResult_Response__Sequence__init(interface__action__Move_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_GetResult_Response * data = NULL;

  if (size) {
    data = (interface__action__Move_GetResult_Response *)allocator.zero_allocate(size, sizeof(interface__action__Move_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface__action__Move_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface__action__Move_GetResult_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
interface__action__Move_GetResult_Response__Sequence__fini(interface__action__Move_GetResult_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      interface__action__Move_GetResult_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

interface__action__Move_GetResult_Response__Sequence *
interface__action__Move_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_GetResult_Response__Sequence * array = (interface__action__Move_GetResult_Response__Sequence *)allocator.allocate(sizeof(interface__action__Move_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface__action__Move_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface__action__Move_GetResult_Response__Sequence__destroy(interface__action__Move_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface__action__Move_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface__action__Move_GetResult_Response__Sequence__are_equal(const interface__action__Move_GetResult_Response__Sequence * lhs, const interface__action__Move_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface__action__Move_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface__action__Move_GetResult_Response__Sequence__copy(
  const interface__action__Move_GetResult_Response__Sequence * input,
  interface__action__Move_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface__action__Move_GetResult_Response);
    interface__action__Move_GetResult_Response * data =
      (interface__action__Move_GetResult_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface__action__Move_GetResult_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          interface__action__Move_GetResult_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface__action__Move_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "interface/action/detail/move__functions.h"

bool
interface__action__Move_FeedbackMessage__init(interface__action__Move_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    interface__action__Move_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!interface__action__Move_Feedback__init(&msg->feedback)) {
    interface__action__Move_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
interface__action__Move_FeedbackMessage__fini(interface__action__Move_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  interface__action__Move_Feedback__fini(&msg->feedback);
}

bool
interface__action__Move_FeedbackMessage__are_equal(const interface__action__Move_FeedbackMessage * lhs, const interface__action__Move_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!interface__action__Move_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
interface__action__Move_FeedbackMessage__copy(
  const interface__action__Move_FeedbackMessage * input,
  interface__action__Move_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!interface__action__Move_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

interface__action__Move_FeedbackMessage *
interface__action__Move_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_FeedbackMessage * msg = (interface__action__Move_FeedbackMessage *)allocator.allocate(sizeof(interface__action__Move_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface__action__Move_FeedbackMessage));
  bool success = interface__action__Move_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface__action__Move_FeedbackMessage__destroy(interface__action__Move_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface__action__Move_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface__action__Move_FeedbackMessage__Sequence__init(interface__action__Move_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_FeedbackMessage * data = NULL;

  if (size) {
    data = (interface__action__Move_FeedbackMessage *)allocator.zero_allocate(size, sizeof(interface__action__Move_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface__action__Move_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface__action__Move_FeedbackMessage__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
interface__action__Move_FeedbackMessage__Sequence__fini(interface__action__Move_FeedbackMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      interface__action__Move_FeedbackMessage__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

interface__action__Move_FeedbackMessage__Sequence *
interface__action__Move_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__action__Move_FeedbackMessage__Sequence * array = (interface__action__Move_FeedbackMessage__Sequence *)allocator.allocate(sizeof(interface__action__Move_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface__action__Move_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface__action__Move_FeedbackMessage__Sequence__destroy(interface__action__Move_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface__action__Move_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface__action__Move_FeedbackMessage__Sequence__are_equal(const interface__action__Move_FeedbackMessage__Sequence * lhs, const interface__action__Move_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface__action__Move_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface__action__Move_FeedbackMessage__Sequence__copy(
  const interface__action__Move_FeedbackMessage__Sequence * input,
  interface__action__Move_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface__action__Move_FeedbackMessage);
    interface__action__Move_FeedbackMessage * data =
      (interface__action__Move_FeedbackMessage *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface__action__Move_FeedbackMessage__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          interface__action__Move_FeedbackMessage__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface__action__Move_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
