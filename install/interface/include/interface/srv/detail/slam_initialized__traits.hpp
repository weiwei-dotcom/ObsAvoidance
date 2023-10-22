// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface:srv/SlamInitialized.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__SLAM_INITIALIZED__TRAITS_HPP_
#define INTERFACE__SRV__DETAIL__SLAM_INITIALIZED__TRAITS_HPP_

#include "interface/srv/detail/slam_initialized__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::SlamInitialized_Request>()
{
  return "interface::srv::SlamInitialized_Request";
}

template<>
inline const char * name<interface::srv::SlamInitialized_Request>()
{
  return "interface/srv/SlamInitialized_Request";
}

template<>
struct has_fixed_size<interface::srv::SlamInitialized_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interface::srv::SlamInitialized_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interface::srv::SlamInitialized_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::SlamInitialized_Response>()
{
  return "interface::srv::SlamInitialized_Response";
}

template<>
inline const char * name<interface::srv::SlamInitialized_Response>()
{
  return "interface/srv/SlamInitialized_Response";
}

template<>
struct has_fixed_size<interface::srv::SlamInitialized_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interface::srv::SlamInitialized_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interface::srv::SlamInitialized_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::SlamInitialized>()
{
  return "interface::srv::SlamInitialized";
}

template<>
inline const char * name<interface::srv::SlamInitialized>()
{
  return "interface/srv/SlamInitialized";
}

template<>
struct has_fixed_size<interface::srv::SlamInitialized>
  : std::integral_constant<
    bool,
    has_fixed_size<interface::srv::SlamInitialized_Request>::value &&
    has_fixed_size<interface::srv::SlamInitialized_Response>::value
  >
{
};

template<>
struct has_bounded_size<interface::srv::SlamInitialized>
  : std::integral_constant<
    bool,
    has_bounded_size<interface::srv::SlamInitialized_Request>::value &&
    has_bounded_size<interface::srv::SlamInitialized_Response>::value
  >
{
};

template<>
struct is_service<interface::srv::SlamInitialized>
  : std::true_type
{
};

template<>
struct is_service_request<interface::srv::SlamInitialized_Request>
  : std::true_type
{
};

template<>
struct is_service_response<interface::srv::SlamInitialized_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE__SRV__DETAIL__SLAM_INITIALIZED__TRAITS_HPP_
