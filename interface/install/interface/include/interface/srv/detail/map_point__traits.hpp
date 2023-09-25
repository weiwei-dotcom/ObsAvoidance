// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface:srv/MapPoint.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__MAP_POINT__TRAITS_HPP_
#define INTERFACE__SRV__DETAIL__MAP_POINT__TRAITS_HPP_

#include "interface/srv/detail/map_point__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::MapPoint_Request>()
{
  return "interface::srv::MapPoint_Request";
}

template<>
inline const char * name<interface::srv::MapPoint_Request>()
{
  return "interface/srv/MapPoint_Request";
}

template<>
struct has_fixed_size<interface::srv::MapPoint_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interface::srv::MapPoint_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interface::srv::MapPoint_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'point_cloud'
#include "sensor_msgs/msg/detail/point_cloud2__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::MapPoint_Response>()
{
  return "interface::srv::MapPoint_Response";
}

template<>
inline const char * name<interface::srv::MapPoint_Response>()
{
  return "interface/srv/MapPoint_Response";
}

template<>
struct has_fixed_size<interface::srv::MapPoint_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interface::srv::MapPoint_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interface::srv::MapPoint_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::MapPoint>()
{
  return "interface::srv::MapPoint";
}

template<>
inline const char * name<interface::srv::MapPoint>()
{
  return "interface/srv/MapPoint";
}

template<>
struct has_fixed_size<interface::srv::MapPoint>
  : std::integral_constant<
    bool,
    has_fixed_size<interface::srv::MapPoint_Request>::value &&
    has_fixed_size<interface::srv::MapPoint_Response>::value
  >
{
};

template<>
struct has_bounded_size<interface::srv::MapPoint>
  : std::integral_constant<
    bool,
    has_bounded_size<interface::srv::MapPoint_Request>::value &&
    has_bounded_size<interface::srv::MapPoint_Response>::value
  >
{
};

template<>
struct is_service<interface::srv::MapPoint>
  : std::true_type
{
};

template<>
struct is_service_request<interface::srv::MapPoint_Request>
  : std::true_type
{
};

template<>
struct is_service_response<interface::srv::MapPoint_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE__SRV__DETAIL__MAP_POINT__TRAITS_HPP_
