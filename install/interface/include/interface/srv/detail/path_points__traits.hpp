// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface:srv/PathPoints.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__PATH_POINTS__TRAITS_HPP_
#define INTERFACE__SRV__DETAIL__PATH_POINTS__TRAITS_HPP_

#include "interface/srv/detail/path_points__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'start_position'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'start_velocity'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::PathPoints_Request>()
{
  return "interface::srv::PathPoints_Request";
}

template<>
inline const char * name<interface::srv::PathPoints_Request>()
{
  return "interface/srv/PathPoints_Request";
}

template<>
struct has_fixed_size<interface::srv::PathPoints_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<interface::srv::PathPoints_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<interface::srv::PathPoints_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::PathPoints_Response>()
{
  return "interface::srv::PathPoints_Response";
}

template<>
inline const char * name<interface::srv::PathPoints_Response>()
{
  return "interface/srv/PathPoints_Response";
}

template<>
struct has_fixed_size<interface::srv::PathPoints_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interface::srv::PathPoints_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interface::srv::PathPoints_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::PathPoints>()
{
  return "interface::srv::PathPoints";
}

template<>
inline const char * name<interface::srv::PathPoints>()
{
  return "interface/srv/PathPoints";
}

template<>
struct has_fixed_size<interface::srv::PathPoints>
  : std::integral_constant<
    bool,
    has_fixed_size<interface::srv::PathPoints_Request>::value &&
    has_fixed_size<interface::srv::PathPoints_Response>::value
  >
{
};

template<>
struct has_bounded_size<interface::srv::PathPoints>
  : std::integral_constant<
    bool,
    has_bounded_size<interface::srv::PathPoints_Request>::value &&
    has_bounded_size<interface::srv::PathPoints_Response>::value
  >
{
};

template<>
struct is_service<interface::srv::PathPoints>
  : std::true_type
{
};

template<>
struct is_service_request<interface::srv::PathPoints_Request>
  : std::true_type
{
};

template<>
struct is_service_response<interface::srv::PathPoints_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE__SRV__DETAIL__PATH_POINTS__TRAITS_HPP_
