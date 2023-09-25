// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface:srv/CamPose.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__CAM_POSE__TRAITS_HPP_
#define INTERFACE__SRV__DETAIL__CAM_POSE__TRAITS_HPP_

#include "interface/srv/detail/cam_pose__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::CamPose_Request>()
{
  return "interface::srv::CamPose_Request";
}

template<>
inline const char * name<interface::srv::CamPose_Request>()
{
  return "interface/srv/CamPose_Request";
}

template<>
struct has_fixed_size<interface::srv::CamPose_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interface::srv::CamPose_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interface::srv::CamPose_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'cam_pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::CamPose_Response>()
{
  return "interface::srv::CamPose_Response";
}

template<>
inline const char * name<interface::srv::CamPose_Response>()
{
  return "interface/srv/CamPose_Response";
}

template<>
struct has_fixed_size<interface::srv::CamPose_Response>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<interface::srv::CamPose_Response>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<interface::srv::CamPose_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::srv::CamPose>()
{
  return "interface::srv::CamPose";
}

template<>
inline const char * name<interface::srv::CamPose>()
{
  return "interface/srv/CamPose";
}

template<>
struct has_fixed_size<interface::srv::CamPose>
  : std::integral_constant<
    bool,
    has_fixed_size<interface::srv::CamPose_Request>::value &&
    has_fixed_size<interface::srv::CamPose_Response>::value
  >
{
};

template<>
struct has_bounded_size<interface::srv::CamPose>
  : std::integral_constant<
    bool,
    has_bounded_size<interface::srv::CamPose_Request>::value &&
    has_bounded_size<interface::srv::CamPose_Response>::value
  >
{
};

template<>
struct is_service<interface::srv::CamPose>
  : std::true_type
{
};

template<>
struct is_service_request<interface::srv::CamPose_Request>
  : std::true_type
{
};

template<>
struct is_service_response<interface::srv::CamPose_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE__SRV__DETAIL__CAM_POSE__TRAITS_HPP_
