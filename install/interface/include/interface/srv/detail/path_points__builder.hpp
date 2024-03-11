// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface:srv/PathPoints.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__PATH_POINTS__BUILDER_HPP_
#define INTERFACE__SRV__DETAIL__PATH_POINTS__BUILDER_HPP_

#include "interface/srv/detail/path_points__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace interface
{

namespace srv
{

namespace builder
{

class Init_PathPoints_Request_start_velocity
{
public:
  explicit Init_PathPoints_Request_start_velocity(::interface::srv::PathPoints_Request & msg)
  : msg_(msg)
  {}
  ::interface::srv::PathPoints_Request start_velocity(::interface::srv::PathPoints_Request::_start_velocity_type arg)
  {
    msg_.start_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::srv::PathPoints_Request msg_;
};

class Init_PathPoints_Request_start_position
{
public:
  Init_PathPoints_Request_start_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathPoints_Request_start_velocity start_position(::interface::srv::PathPoints_Request::_start_position_type arg)
  {
    msg_.start_position = std::move(arg);
    return Init_PathPoints_Request_start_velocity(msg_);
  }

private:
  ::interface::srv::PathPoints_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::srv::PathPoints_Request>()
{
  return interface::srv::builder::Init_PathPoints_Request_start_position();
}

}  // namespace interface


namespace interface
{

namespace srv
{

namespace builder
{

class Init_PathPoints_Response_success
{
public:
  explicit Init_PathPoints_Response_success(::interface::srv::PathPoints_Response & msg)
  : msg_(msg)
  {}
  ::interface::srv::PathPoints_Response success(::interface::srv::PathPoints_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::srv::PathPoints_Response msg_;
};

class Init_PathPoints_Response_path_points
{
public:
  Init_PathPoints_Response_path_points()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathPoints_Response_success path_points(::interface::srv::PathPoints_Response::_path_points_type arg)
  {
    msg_.path_points = std::move(arg);
    return Init_PathPoints_Response_success(msg_);
  }

private:
  ::interface::srv::PathPoints_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::srv::PathPoints_Response>()
{
  return interface::srv::builder::Init_PathPoints_Response_path_points();
}

}  // namespace interface

#endif  // INTERFACE__SRV__DETAIL__PATH_POINTS__BUILDER_HPP_
