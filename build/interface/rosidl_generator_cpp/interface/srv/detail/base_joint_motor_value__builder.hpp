// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface:srv/BaseJointMotorValue.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__BASE_JOINT_MOTOR_VALUE__BUILDER_HPP_
#define INTERFACE__SRV__DETAIL__BASE_JOINT_MOTOR_VALUE__BUILDER_HPP_

#include "interface/srv/detail/base_joint_motor_value__struct.hpp"
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
auto build<::interface::srv::BaseJointMotorValue_Request>()
{
  return ::interface::srv::BaseJointMotorValue_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace interface


namespace interface
{

namespace srv
{

namespace builder
{

class Init_BaseJointMotorValue_Response_flag_arrived_target
{
public:
  explicit Init_BaseJointMotorValue_Response_flag_arrived_target(::interface::srv::BaseJointMotorValue_Response & msg)
  : msg_(msg)
  {}
  ::interface::srv::BaseJointMotorValue_Response flag_arrived_target(::interface::srv::BaseJointMotorValue_Response::_flag_arrived_target_type arg)
  {
    msg_.flag_arrived_target = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::srv::BaseJointMotorValue_Response msg_;
};

class Init_BaseJointMotorValue_Response_flag_base_out_range
{
public:
  explicit Init_BaseJointMotorValue_Response_flag_base_out_range(::interface::srv::BaseJointMotorValue_Response & msg)
  : msg_(msg)
  {}
  Init_BaseJointMotorValue_Response_flag_arrived_target flag_base_out_range(::interface::srv::BaseJointMotorValue_Response::_flag_base_out_range_type arg)
  {
    msg_.flag_base_out_range = std::move(arg);
    return Init_BaseJointMotorValue_Response_flag_arrived_target(msg_);
  }

private:
  ::interface::srv::BaseJointMotorValue_Response msg_;
};

class Init_BaseJointMotorValue_Response_base_advance_value
{
public:
  explicit Init_BaseJointMotorValue_Response_base_advance_value(::interface::srv::BaseJointMotorValue_Response & msg)
  : msg_(msg)
  {}
  Init_BaseJointMotorValue_Response_flag_base_out_range base_advance_value(::interface::srv::BaseJointMotorValue_Response::_base_advance_value_type arg)
  {
    msg_.base_advance_value = std::move(arg);
    return Init_BaseJointMotorValue_Response_flag_base_out_range(msg_);
  }

private:
  ::interface::srv::BaseJointMotorValue_Response msg_;
};

class Init_BaseJointMotorValue_Response_cable_expend_value
{
public:
  Init_BaseJointMotorValue_Response_cable_expend_value()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BaseJointMotorValue_Response_base_advance_value cable_expend_value(::interface::srv::BaseJointMotorValue_Response::_cable_expend_value_type arg)
  {
    msg_.cable_expend_value = std::move(arg);
    return Init_BaseJointMotorValue_Response_base_advance_value(msg_);
  }

private:
  ::interface::srv::BaseJointMotorValue_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::srv::BaseJointMotorValue_Response>()
{
  return interface::srv::builder::Init_BaseJointMotorValue_Response_cable_expend_value();
}

}  // namespace interface

#endif  // INTERFACE__SRV__DETAIL__BASE_JOINT_MOTOR_VALUE__BUILDER_HPP_
