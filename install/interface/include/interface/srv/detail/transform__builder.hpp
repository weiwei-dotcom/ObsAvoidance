// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface:srv/Transform.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__TRANSFORM__BUILDER_HPP_
#define INTERFACE__SRV__DETAIL__TRANSFORM__BUILDER_HPP_

#include "interface/srv/detail/transform__struct.hpp"
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
auto build<::interface::srv::Transform_Request>()
{
  return ::interface::srv::Transform_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace interface


namespace interface
{

namespace srv
{

namespace builder
{

class Init_Transform_Response_transform_base_to_world
{
public:
  explicit Init_Transform_Response_transform_base_to_world(::interface::srv::Transform_Response & msg)
  : msg_(msg)
  {}
  ::interface::srv::Transform_Response transform_base_to_world(::interface::srv::Transform_Response::_transform_base_to_world_type arg)
  {
    msg_.transform_base_to_world = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::srv::Transform_Response msg_;
};

class Init_Transform_Response_transform_init_to_world
{
public:
  Init_Transform_Response_transform_init_to_world()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Transform_Response_transform_base_to_world transform_init_to_world(::interface::srv::Transform_Response::_transform_init_to_world_type arg)
  {
    msg_.transform_init_to_world = std::move(arg);
    return Init_Transform_Response_transform_base_to_world(msg_);
  }

private:
  ::interface::srv::Transform_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::srv::Transform_Response>()
{
  return interface::srv::builder::Init_Transform_Response_transform_init_to_world();
}

}  // namespace interface

#endif  // INTERFACE__SRV__DETAIL__TRANSFORM__BUILDER_HPP_
