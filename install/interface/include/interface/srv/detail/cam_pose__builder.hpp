// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface:srv/CamPose.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__CAM_POSE__BUILDER_HPP_
#define INTERFACE__SRV__DETAIL__CAM_POSE__BUILDER_HPP_

#include "interface/srv/detail/cam_pose__struct.hpp"
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
auto build<::interface::srv::CamPose_Request>()
{
  return ::interface::srv::CamPose_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace interface


namespace interface
{

namespace srv
{

namespace builder
{

class Init_CamPose_Response_cam_pose
{
public:
  Init_CamPose_Response_cam_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interface::srv::CamPose_Response cam_pose(::interface::srv::CamPose_Response::_cam_pose_type arg)
  {
    msg_.cam_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::srv::CamPose_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::srv::CamPose_Response>()
{
  return interface::srv::builder::Init_CamPose_Response_cam_pose();
}

}  // namespace interface

#endif  // INTERFACE__SRV__DETAIL__CAM_POSE__BUILDER_HPP_
