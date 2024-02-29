// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface:srv/EnableFollow.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__ENABLE_FOLLOW__TRAITS_HPP_
#define INTERFACE__SRV__DETAIL__ENABLE_FOLLOW__TRAITS_HPP_

#include "interface/srv/detail/enable_follow__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::EnableFollow_Request>()
{
  return "interface::srv::EnableFollow_Request";
}

template<>
inline const char * name<interface::srv::EnableFollow_Request>()
{
  return "interface/srv/EnableFollow_Request";
}

template<>
struct has_fixed_size<interface::srv::EnableFollow_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interface::srv::EnableFollow_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interface::srv::EnableFollow_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::EnableFollow_Response>()
{
  return "interface::srv::EnableFollow_Response";
}

template<>
inline const char * name<interface::srv::EnableFollow_Response>()
{
  return "interface/srv/EnableFollow_Response";
}

template<>
struct has_fixed_size<interface::srv::EnableFollow_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interface::srv::EnableFollow_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interface::srv::EnableFollow_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::EnableFollow>()
{
  return "interface::srv::EnableFollow";
}

template<>
inline const char * name<interface::srv::EnableFollow>()
{
  return "interface/srv/EnableFollow";
}

template<>
struct has_fixed_size<interface::srv::EnableFollow>
  : std::integral_constant<
    bool,
    has_fixed_size<interface::srv::EnableFollow_Request>::value &&
    has_fixed_size<interface::srv::EnableFollow_Response>::value
  >
{
};

template<>
struct has_bounded_size<interface::srv::EnableFollow>
  : std::integral_constant<
    bool,
    has_bounded_size<interface::srv::EnableFollow_Request>::value &&
    has_bounded_size<interface::srv::EnableFollow_Response>::value
  >
{
};

template<>
struct is_service<interface::srv::EnableFollow>
  : std::true_type
{
};

template<>
struct is_service_request<interface::srv::EnableFollow_Request>
  : std::true_type
{
};

template<>
struct is_service_response<interface::srv::EnableFollow_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE__SRV__DETAIL__ENABLE_FOLLOW__TRAITS_HPP_
