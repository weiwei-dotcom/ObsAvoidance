// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface:srv/T.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__T__BUILDER_HPP_
#define INTERFACE__SRV__DETAIL__T__BUILDER_HPP_

#include "interface/srv/detail/t__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace interface
{

namespace srv
{

namespace builder
{

class Init_T_Request_request_flag
{
public:
  Init_T_Request_request_flag()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interface::srv::T_Request request_flag(::interface::srv::T_Request::_request_flag_type arg)
  {
    msg_.request_flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::srv::T_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::srv::T_Request>()
{
  return interface::srv::builder::Init_T_Request_request_flag();
}

}  // namespace interface


namespace interface
{

namespace srv
{

namespace builder
{

class Init_T_Response_transform_base_to_world
{
public:
  explicit Init_T_Response_transform_base_to_world(::interface::srv::T_Response & msg)
  : msg_(msg)
  {}
  ::interface::srv::T_Response transform_base_to_world(::interface::srv::T_Response::_transform_base_to_world_type arg)
  {
    msg_.transform_base_to_world = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::srv::T_Response msg_;
};

class Init_T_Response_transform_init_to_world
{
public:
  Init_T_Response_transform_init_to_world()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_T_Response_transform_base_to_world transform_init_to_world(::interface::srv::T_Response::_transform_init_to_world_type arg)
  {
    msg_.transform_init_to_world = std::move(arg);
    return Init_T_Response_transform_base_to_world(msg_);
  }

private:
  ::interface::srv::T_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::srv::T_Response>()
{
  return interface::srv::builder::Init_T_Response_transform_init_to_world();
}

}  // namespace interface

#endif  // INTERFACE__SRV__DETAIL__T__BUILDER_HPP_
