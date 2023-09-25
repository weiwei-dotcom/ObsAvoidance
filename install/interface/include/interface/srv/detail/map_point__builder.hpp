// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface:srv/MapPoint.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__MAP_POINT__BUILDER_HPP_
#define INTERFACE__SRV__DETAIL__MAP_POINT__BUILDER_HPP_

#include "interface/srv/detail/map_point__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace interface
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::srv::MapPoint_Request>()
{
  return ::interface::srv::MapPoint_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace interface


namespace interface
{

namespace srv
{

namespace builder
{

class Init_MapPoint_Response_world2cam
{
public:
  explicit Init_MapPoint_Response_world2cam(::interface::srv::MapPoint_Response & msg)
  : msg_(msg)
  {}
  ::interface::srv::MapPoint_Response world2cam(::interface::srv::MapPoint_Response::_world2cam_type arg)
  {
    msg_.world2cam = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::srv::MapPoint_Response msg_;
};

class Init_MapPoint_Response_cam_pose
{
public:
  explicit Init_MapPoint_Response_cam_pose(::interface::srv::MapPoint_Response & msg)
  : msg_(msg)
  {}
  Init_MapPoint_Response_world2cam cam_pose(::interface::srv::MapPoint_Response::_cam_pose_type arg)
  {
    msg_.cam_pose = std::move(arg);
    return Init_MapPoint_Response_world2cam(msg_);
  }

private:
  ::interface::srv::MapPoint_Response msg_;
};

class Init_MapPoint_Response_img
{
public:
  explicit Init_MapPoint_Response_img(::interface::srv::MapPoint_Response & msg)
  : msg_(msg)
  {}
  Init_MapPoint_Response_cam_pose img(::interface::srv::MapPoint_Response::_img_type arg)
  {
    msg_.img = std::move(arg);
    return Init_MapPoint_Response_cam_pose(msg_);
  }

private:
  ::interface::srv::MapPoint_Response msg_;
};

class Init_MapPoint_Response_point_cloud
{
public:
  Init_MapPoint_Response_point_cloud()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MapPoint_Response_img point_cloud(::interface::srv::MapPoint_Response::_point_cloud_type arg)
  {
    msg_.point_cloud = std::move(arg);
    return Init_MapPoint_Response_img(msg_);
  }

private:
  ::interface::srv::MapPoint_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::srv::MapPoint_Response>()
{
  return interface::srv::builder::Init_MapPoint_Response_point_cloud();
}

}  // namespace interface

#endif  // INTERFACE__SRV__DETAIL__MAP_POINT__BUILDER_HPP_
