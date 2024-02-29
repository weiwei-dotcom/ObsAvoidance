// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface:srv/EnableFollow.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__ENABLE_FOLLOW__BUILDER_HPP_
#define INTERFACE__SRV__DETAIL__ENABLE_FOLLOW__BUILDER_HPP_

#include "interface/srv/detail/enable_follow__struct.hpp"
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
auto build<::interface::srv::EnableFollow_Request>()
{
  return ::interface::srv::EnableFollow_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace interface


namespace interface
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::srv::EnableFollow_Response>()
{
  return ::interface::srv::EnableFollow_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace interface

#endif  // INTERFACE__SRV__DETAIL__ENABLE_FOLLOW__BUILDER_HPP_
