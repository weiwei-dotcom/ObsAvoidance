// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface:srv/T.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__T__TRAITS_HPP_
#define INTERFACE__SRV__DETAIL__T__TRAITS_HPP_

#include "interface/srv/detail/t__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::T_Request>()
{
  return "interface::srv::T_Request";
}

template<>
inline const char * name<interface::srv::T_Request>()
{
  return "interface/srv/T_Request";
}

template<>
struct has_fixed_size<interface::srv::T_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interface::srv::T_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interface::srv::T_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'transform_init_to_world'
// Member 'transform_base_to_world'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::T_Response>()
{
  return "interface::srv::T_Response";
}

template<>
inline const char * name<interface::srv::T_Response>()
{
  return "interface/srv/T_Response";
}

template<>
struct has_fixed_size<interface::srv::T_Response>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<interface::srv::T_Response>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<interface::srv::T_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::T>()
{
  return "interface::srv::T";
}

template<>
inline const char * name<interface::srv::T>()
{
  return "interface/srv/T";
}

template<>
struct has_fixed_size<interface::srv::T>
  : std::integral_constant<
    bool,
    has_fixed_size<interface::srv::T_Request>::value &&
    has_fixed_size<interface::srv::T_Response>::value
  >
{
};

template<>
struct has_bounded_size<interface::srv::T>
  : std::integral_constant<
    bool,
    has_bounded_size<interface::srv::T_Request>::value &&
    has_bounded_size<interface::srv::T_Response>::value
  >
{
};

template<>
struct is_service<interface::srv::T>
  : std::true_type
{
};

template<>
struct is_service_request<interface::srv::T_Request>
  : std::true_type
{
};

template<>
struct is_service_response<interface::srv::T_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE__SRV__DETAIL__T__TRAITS_HPP_
