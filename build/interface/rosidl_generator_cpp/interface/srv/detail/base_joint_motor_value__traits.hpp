// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface:srv/BaseJointMotorValue.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__BASE_JOINT_MOTOR_VALUE__TRAITS_HPP_
#define INTERFACE__SRV__DETAIL__BASE_JOINT_MOTOR_VALUE__TRAITS_HPP_

#include "interface/srv/detail/base_joint_motor_value__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::BaseJointMotorValue_Request>()
{
  return "interface::srv::BaseJointMotorValue_Request";
}

template<>
inline const char * name<interface::srv::BaseJointMotorValue_Request>()
{
  return "interface/srv/BaseJointMotorValue_Request";
}

template<>
struct has_fixed_size<interface::srv::BaseJointMotorValue_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interface::srv::BaseJointMotorValue_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interface::srv::BaseJointMotorValue_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::BaseJointMotorValue_Response>()
{
  return "interface::srv::BaseJointMotorValue_Response";
}

template<>
inline const char * name<interface::srv::BaseJointMotorValue_Response>()
{
  return "interface/srv/BaseJointMotorValue_Response";
}

template<>
struct has_fixed_size<interface::srv::BaseJointMotorValue_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interface::srv::BaseJointMotorValue_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interface::srv::BaseJointMotorValue_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::BaseJointMotorValue>()
{
  return "interface::srv::BaseJointMotorValue";
}

template<>
inline const char * name<interface::srv::BaseJointMotorValue>()
{
  return "interface/srv/BaseJointMotorValue";
}

template<>
struct has_fixed_size<interface::srv::BaseJointMotorValue>
  : std::integral_constant<
    bool,
    has_fixed_size<interface::srv::BaseJointMotorValue_Request>::value &&
    has_fixed_size<interface::srv::BaseJointMotorValue_Response>::value
  >
{
};

template<>
struct has_bounded_size<interface::srv::BaseJointMotorValue>
  : std::integral_constant<
    bool,
    has_bounded_size<interface::srv::BaseJointMotorValue_Request>::value &&
    has_bounded_size<interface::srv::BaseJointMotorValue_Response>::value
  >
{
};

template<>
struct is_service<interface::srv::BaseJointMotorValue>
  : std::true_type
{
};

template<>
struct is_service_request<interface::srv::BaseJointMotorValue_Request>
  : std::true_type
{
};

template<>
struct is_service_response<interface::srv::BaseJointMotorValue_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE__SRV__DETAIL__BASE_JOINT_MOTOR_VALUE__TRAITS_HPP_
