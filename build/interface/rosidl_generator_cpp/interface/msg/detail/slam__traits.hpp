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
// Member 'point_cloud'
#include "sensor_msgs/msg/detail/point_cloud2__traits.hpp"
// Member 'cam_pose'
// Member 'world2cam'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'img'
#include "sensor_msgs/msg/detail/image__traits.hpp"

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
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<sensor_msgs::msg::Image>::value && has_fixed_size<sensor_msgs::msg::PointCloud2>::value> {};

template<>
struct has_bounded_size<interface::msg::Slam>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<sensor_msgs::msg::Image>::value && has_bounded_size<sensor_msgs::msg::PointCloud2>::value> {};

template<>
struct is_message<interface::msg::Slam>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE__MSG__DETAIL__SLAM__TRAITS_HPP_
