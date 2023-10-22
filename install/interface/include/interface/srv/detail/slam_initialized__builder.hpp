// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface:srv/SlamInitialized.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__SLAM_INITIALIZED__BUILDER_HPP_
#define INTERFACE__SRV__DETAIL__SLAM_INITIALIZED__BUILDER_HPP_

#include "interface/srv/detail/slam_initialized__struct.hpp"
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
auto build<::interface::srv::SlamInitialized_Request>()
{
  return ::interface::srv::SlamInitialized_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace interface


namespace interface
{

namespace srv
{

namespace builder
{

class Init_SlamInitialized_Response_flag_slam_initialized
{
public:
  Init_SlamInitialized_Response_flag_slam_initialized()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interface::srv::SlamInitialized_Response flag_slam_initialized(::interface::srv::SlamInitialized_Response::_flag_slam_initialized_type arg)
  {
    msg_.flag_slam_initialized = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::srv::SlamInitialized_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::srv::SlamInitialized_Response>()
{
  return interface::srv::builder::Init_SlamInitialized_Response_flag_slam_initialized();
}

}  // namespace interface

#endif  // INTERFACE__SRV__DETAIL__SLAM_INITIALIZED__BUILDER_HPP_
