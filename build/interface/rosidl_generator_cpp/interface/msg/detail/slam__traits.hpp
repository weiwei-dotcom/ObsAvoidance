// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface:msg/Slam.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__MSG__DETAIL__SLAM__TRAITS_HPP_
#define INTERFACE__MSG__DETAIL__SLAM__TRAITS_HPP_

#include "interface/msg/detail/slam__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'point_cloud'
#include "sensor_msgs/msg/detail/point_cloud2__traits.hpp"
// Member 'transform_init2cur'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface::msg::Slam>()
{
  return "interface::msg::Slam";
}

template<>
inline const char * name<interface::msg::Slam>()
{
  return "interface/msg/Slam";
}

template<>
struct has_fixed_size<interface::msg::Slam>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value && has_fixed_size<sensor_msgs::msg::PointCloud2>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<interface::msg::Slam>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value && has_bounded_size<sensor_msgs::msg::PointCloud2>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<interface::msg::Slam>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE__MSG__DETAIL__SLAM__TRAITS_HPP_
