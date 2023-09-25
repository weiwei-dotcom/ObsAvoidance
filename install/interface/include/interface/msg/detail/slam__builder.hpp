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

class Init_Slam_img
{
public:
  explicit Init_Slam_img(::interface::msg::Slam & msg)
  : msg_(msg)
  {}
  ::interface::msg::Slam img(::interface::msg::Slam::_img_type arg)
  {
    msg_.img = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::msg::Slam msg_;
};

class Init_Slam_world2cam
{
public:
  explicit Init_Slam_world2cam(::interface::msg::Slam & msg)
  : msg_(msg)
  {}
  Init_Slam_img world2cam(::interface::msg::Slam::_world2cam_type arg)
  {
    msg_.world2cam = std::move(arg);
    return Init_Slam_img(msg_);
  }

private:
  ::interface::msg::Slam msg_;
};

class Init_Slam_cam_pose
{
public:
  explicit Init_Slam_cam_pose(::interface::msg::Slam & msg)
  : msg_(msg)
  {}
  Init_Slam_world2cam cam_pose(::interface::msg::Slam::_cam_pose_type arg)
  {
    msg_.cam_pose = std::move(arg);
    return Init_Slam_world2cam(msg_);
  }

private:
  ::interface::msg::Slam msg_;
};

class Init_Slam_point_cloud
{
public:
  Init_Slam_point_cloud()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Slam_cam_pose point_cloud(::interface::msg::Slam::_point_cloud_type arg)
  {
    msg_.point_cloud = std::move(arg);
    return Init_Slam_cam_pose(msg_);
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
  return interface::msg::builder::Init_Slam_point_cloud();
}

}  // namespace interface

#endif  // INTERFACE__MSG__DETAIL__SLAM__BUILDER_HPP_
