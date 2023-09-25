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

namespace builder
{

class Init_MapPoint_Request_flag
{
public:
  Init_MapPoint_Request_flag()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interface::srv::MapPoint_Request flag(::interface::srv::MapPoint_Request::_flag_type arg)
  {
    msg_.flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::srv::MapPoint_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::srv::MapPoint_Request>()
{
  return interface::srv::builder::Init_MapPoint_Request_flag();
}

}  // namespace interface


namespace interface
{

namespace srv
{

namespace builder
{

class Init_MapPoint_Response_success
{
public:
  explicit Init_MapPoint_Response_success(::interface::srv::MapPoint_Response & msg)
  : msg_(msg)
  {}
  ::interface::srv::MapPoint_Response success(::interface::srv::MapPoint_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::srv::MapPoint_Response msg_;
};

class Init_MapPoint_Response_keypoints
{
public:
  explicit Init_MapPoint_Response_keypoints(::interface::srv::MapPoint_Response & msg)
  : msg_(msg)
  {}
  Init_MapPoint_Response_success keypoints(::interface::srv::MapPoint_Response::_keypoints_type arg)
  {
    msg_.keypoints = std::move(arg);
    return Init_MapPoint_Response_success(msg_);
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
  Init_MapPoint_Response_keypoints point_cloud(::interface::srv::MapPoint_Response::_point_cloud_type arg)
  {
    msg_.point_cloud = std::move(arg);
    return Init_MapPoint_Response_keypoints(msg_);
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
