// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interface:srv/Transform.idl
// generated code does not contain a copyright notice
#include "interface/srv/detail/transform__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
interface__srv__Transform_Request__init(interface__srv__Transform_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
interface__srv__Transform_Request__fini(interface__srv__Transform_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
interface__srv__Transform_Request__are_equal(const interface__srv__Transform_Request * lhs, const interface__srv__Transform_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
interface__srv__Transform_Request__copy(
  const interface__srv__Transform_Request * input,
  interface__srv__Transform_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

interface__srv__Transform_Request *
interface__srv__Transform_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__srv__Transform_Request * msg = (interface__srv__Transform_Request *)allocator.allocate(sizeof(interface__srv__Transform_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface__srv__Transform_Request));
  bool success = interface__srv__Transform_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface__srv__Transform_Request__destroy(interface__srv__Transform_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface__srv__Transform_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface__srv__Transform_Request__Sequence__init(interface__srv__Transform_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__srv__Transform_Request * data = NULL;

  if (size) {
    data = (interface__srv__Transform_Request *)allocator.zero_allocate(size, sizeof(interface__srv__Transform_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface__srv__Transform_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface__srv__Transform_Request__fini(&data[i - 1]);
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
interface__srv__Transform_Request__Sequence__fini(interface__srv__Transform_Request__Sequence * array)
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
      interface__srv__Transform_Request__fini(&array->data[i]);
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

interface__srv__Transform_Request__Sequence *
interface__srv__Transform_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__srv__Transform_Request__Sequence * array = (interface__srv__Transform_Request__Sequence *)allocator.allocate(sizeof(interface__srv__Transform_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface__srv__Transform_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface__srv__Transform_Request__Sequence__destroy(interface__srv__Transform_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface__srv__Transform_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface__srv__Transform_Request__Sequence__are_equal(const interface__srv__Transform_Request__Sequence * lhs, const interface__srv__Transform_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface__srv__Transform_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface__srv__Transform_Request__Sequence__copy(
  const interface__srv__Transform_Request__Sequence * input,
  interface__srv__Transform_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface__srv__Transform_Request);
    interface__srv__Transform_Request * data =
      (interface__srv__Transform_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface__srv__Transform_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          interface__srv__Transform_Request__fini(&data[i]);
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
    if (!interface__srv__Transform_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `transform_init_to_world`
// Member `transform_base_to_world`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
interface__srv__Transform_Response__init(interface__srv__Transform_Response * msg)
{
  if (!msg) {
    return false;
  }
  // transform_init_to_world
  if (!geometry_msgs__msg__Pose__init(&msg->transform_init_to_world)) {
    interface__srv__Transform_Response__fini(msg);
    return false;
  }
  // transform_base_to_world
  if (!geometry_msgs__msg__Pose__init(&msg->transform_base_to_world)) {
    interface__srv__Transform_Response__fini(msg);
    return false;
  }
  return true;
}

void
interface__srv__Transform_Response__fini(interface__srv__Transform_Response * msg)
{
  if (!msg) {
    return;
  }
  // transform_init_to_world
  geometry_msgs__msg__Pose__fini(&msg->transform_init_to_world);
  // transform_base_to_world
  geometry_msgs__msg__Pose__fini(&msg->transform_base_to_world);
}

bool
interface__srv__Transform_Response__are_equal(const interface__srv__Transform_Response * lhs, const interface__srv__Transform_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // transform_init_to_world
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->transform_init_to_world), &(rhs->transform_init_to_world)))
  {
    return false;
  }
  // transform_base_to_world
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->transform_base_to_world), &(rhs->transform_base_to_world)))
  {
    return false;
  }
  return true;
}

bool
interface__srv__Transform_Response__copy(
  const interface__srv__Transform_Response * input,
  interface__srv__Transform_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // transform_init_to_world
  if (!geometry_msgs__msg__Pose__copy(
      &(input->transform_init_to_world), &(output->transform_init_to_world)))
  {
    return false;
  }
  // transform_base_to_world
  if (!geometry_msgs__msg__Pose__copy(
      &(input->transform_base_to_world), &(output->transform_base_to_world)))
  {
    return false;
  }
  return true;
}

interface__srv__Transform_Response *
interface__srv__Transform_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__srv__Transform_Response * msg = (interface__srv__Transform_Response *)allocator.allocate(sizeof(interface__srv__Transform_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface__srv__Transform_Response));
  bool success = interface__srv__Transform_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface__srv__Transform_Response__destroy(interface__srv__Transform_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface__srv__Transform_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface__srv__Transform_Response__Sequence__init(interface__srv__Transform_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__srv__Transform_Response * data = NULL;

  if (size) {
    data = (interface__srv__Transform_Response *)allocator.zero_allocate(size, sizeof(interface__srv__Transform_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface__srv__Transform_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface__srv__Transform_Response__fini(&data[i - 1]);
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
interface__srv__Transform_Response__Sequence__fini(interface__srv__Transform_Response__Sequence * array)
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
      interface__srv__Transform_Response__fini(&array->data[i]);
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

interface__srv__Transform_Response__Sequence *
interface__srv__Transform_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__srv__Transform_Response__Sequence * array = (interface__srv__Transform_Response__Sequence *)allocator.allocate(sizeof(interface__srv__Transform_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface__srv__Transform_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface__srv__Transform_Response__Sequence__destroy(interface__srv__Transform_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface__srv__Transform_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface__srv__Transform_Response__Sequence__are_equal(const interface__srv__Transform_Response__Sequence * lhs, const interface__srv__Transform_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface__srv__Transform_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface__srv__Transform_Response__Sequence__copy(
  const interface__srv__Transform_Response__Sequence * input,
  interface__srv__Transform_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface__srv__Transform_Response);
    interface__srv__Transform_Response * data =
      (interface__srv__Transform_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface__srv__Transform_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          interface__srv__Transform_Response__fini(&data[i]);
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
    if (!interface__srv__Transform_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
