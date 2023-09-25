// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface:msg/Slam.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__MSG__DETAIL__SLAM__BUILDER_HPP_
#define INTERFACE__MSG__DETAIL__SLAM__BUILDER_HPP_

#include "interface/msg/detail/slam__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace interface
{

namespace msg
{

namespace builder
{

class Init_Slam_transform_init2cur
{
public:
  explicit Init_Slam_transform_init2cur(::interface::msg::Slam & msg)
  : msg_(msg)
  {}
  ::interface::msg::Slam transform_init2cur(::interface::msg::Slam::_transform_init2cur_type arg)
  {
    msg_.transform_init2cur = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::msg::Slam msg_;
};

class Init_Slam_point_cloud
{
public:
  explicit Init_Slam_point_cloud(::interface::msg::Slam & msg)
  : msg_(msg)
  {}
  Init_Slam_transform_init2cur point_cloud(::interface::msg::Slam::_point_cloud_type arg)
  {
    msg_.point_cloud = std::move(arg);
    return Init_Slam_transform_init2cur(msg_);
  }

private:
  ::interface::msg::Slam msg_;
};

class Init_Slam_header
{
public:
  Init_Slam_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Slam_point_cloud header(::interface::msg::Slam::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Slam_point_cloud(msg_);
  }

private:
  ::interface::msg::Slam msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::msg::Slam>()
{
  return interface::msg::builder::Init_Slam_header();
}

}  // namespace interface

#endif  // INTERFACE__MSG__DETAIL__SLAM__BUILDER_HPP_
