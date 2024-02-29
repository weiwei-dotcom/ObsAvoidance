// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from interface:srv/BaseJointMotorValue.idl
// generated code does not contain a copyright notice
#include "interface/srv/detail/base_joint_motor_value__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "interface/srv/detail/base_joint_motor_value__struct.h"
#include "interface/srv/detail/base_joint_motor_value__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _BaseJointMotorValue_Request__ros_msg_type = interface__srv__BaseJointMotorValue_Request;

static bool _BaseJointMotorValue_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _BaseJointMotorValue_Request__ros_msg_type * ros_message = static_cast<const _BaseJointMotorValue_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr << ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

static bool _BaseJointMotorValue_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _BaseJointMotorValue_Request__ros_msg_type * ros_message = static_cast<_BaseJointMotorValue_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr >> ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface
size_t get_serialized_size_interface__srv__BaseJointMotorValue_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BaseJointMotorValue_Request__ros_msg_type * ros_message = static_cast<const _BaseJointMotorValue_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name structure_needs_at_least_one_member
  {
    size_t item_size = sizeof(ros_message->structure_needs_at_least_one_member);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _BaseJointMotorValue_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_interface__srv__BaseJointMotorValue_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface
size_t max_serialized_size_interface__srv__BaseJointMotorValue_Request(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: structure_needs_at_least_one_member
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _BaseJointMotorValue_Request__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_interface__srv__BaseJointMotorValue_Request(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_BaseJointMotorValue_Request = {
  "interface::srv",
  "BaseJointMotorValue_Request",
  _BaseJointMotorValue_Request__cdr_serialize,
  _BaseJointMotorValue_Request__cdr_deserialize,
  _BaseJointMotorValue_Request__get_serialized_size,
  _BaseJointMotorValue_Request__max_serialized_size
};

static rosidl_message_type_support_t _BaseJointMotorValue_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_BaseJointMotorValue_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interface, srv, BaseJointMotorValue_Request)() {
  return &_BaseJointMotorValue_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "interface/srv/detail/base_joint_motor_value__struct.h"
// already included above
// #include "interface/srv/detail/base_joint_motor_value__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "geometry_msgs/msg/detail/point__functions.h"  // cable_expend_value

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_interface
size_t get_serialized_size_geometry_msgs__msg__Point(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_interface
size_t max_serialized_size_geometry_msgs__msg__Point(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_interface
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Point)();


using _BaseJointMotorValue_Response__ros_msg_type = interface__srv__BaseJointMotorValue_Response;

static bool _BaseJointMotorValue_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _BaseJointMotorValue_Response__ros_msg_type * ros_message = static_cast<const _BaseJointMotorValue_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: cable_expend_value
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Point
      )()->data);
    size_t size = ros_message->cable_expend_value.size;
    auto array_ptr = ros_message->cable_expend_value.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: base_advance_value
  {
    cdr << ros_message->base_advance_value;
  }

  // Field name: flag_base_out_range
  {
    cdr << (ros_message->flag_base_out_range ? true : false);
  }

  // Field name: flag_arrived_target
  {
    cdr << (ros_message->flag_arrived_target ? true : false);
  }

  return true;
}

static bool _BaseJointMotorValue_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _BaseJointMotorValue_Response__ros_msg_type * ros_message = static_cast<_BaseJointMotorValue_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: cable_expend_value
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Point
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->cable_expend_value.data) {
      geometry_msgs__msg__Point__Sequence__fini(&ros_message->cable_expend_value);
    }
    if (!geometry_msgs__msg__Point__Sequence__init(&ros_message->cable_expend_value, size)) {
      return "failed to create array for field 'cable_expend_value'";
    }
    auto array_ptr = ros_message->cable_expend_value.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: base_advance_value
  {
    cdr >> ros_message->base_advance_value;
  }

  // Field name: flag_base_out_range
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->flag_base_out_range = tmp ? true : false;
  }

  // Field name: flag_arrived_target
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->flag_arrived_target = tmp ? true : false;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface
size_t get_serialized_size_interface__srv__BaseJointMotorValue_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BaseJointMotorValue_Response__ros_msg_type * ros_message = static_cast<const _BaseJointMotorValue_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name cable_expend_value
  {
    size_t array_size = ros_message->cable_expend_value.size;
    auto array_ptr = ros_message->cable_expend_value.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_geometry_msgs__msg__Point(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name base_advance_value
  {
    size_t item_size = sizeof(ros_message->base_advance_value);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name flag_base_out_range
  {
    size_t item_size = sizeof(ros_message->flag_base_out_range);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name flag_arrived_target
  {
    size_t item_size = sizeof(ros_message->flag_arrived_target);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _BaseJointMotorValue_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_interface__srv__BaseJointMotorValue_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface
size_t max_serialized_size_interface__srv__BaseJointMotorValue_Response(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: cable_expend_value
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Point(
        full_bounded, current_alignment);
    }
  }
  // member: base_advance_value
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: flag_base_out_range
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: flag_arrived_target
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _BaseJointMotorValue_Response__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_interface__srv__BaseJointMotorValue_Response(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_BaseJointMotorValue_Response = {
  "interface::srv",
  "BaseJointMotorValue_Response",
  _BaseJointMotorValue_Response__cdr_serialize,
  _BaseJointMotorValue_Response__cdr_deserialize,
  _BaseJointMotorValue_Response__get_serialized_size,
  _BaseJointMotorValue_Response__max_serialized_size
};

static rosidl_message_type_support_t _BaseJointMotorValue_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_BaseJointMotorValue_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interface, srv, BaseJointMotorValue_Response)() {
  return &_BaseJointMotorValue_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "interface/srv/base_joint_motor_value.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t BaseJointMotorValue__callbacks = {
  "interface::srv",
  "BaseJointMotorValue",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interface, srv, BaseJointMotorValue_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interface, srv, BaseJointMotorValue_Response)(),
};

static rosidl_service_type_support_t BaseJointMotorValue__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &BaseJointMotorValue__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interface, srv, BaseJointMotorValue)() {
  return &BaseJointMotorValue__handle;
}

#if defined(__cplusplus)
}
#endif
