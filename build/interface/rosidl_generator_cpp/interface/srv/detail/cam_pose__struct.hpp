// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface:srv/CamPose.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__CAM_POSE__STRUCT_HPP_
#define INTERFACE__SRV__DETAIL__CAM_POSE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__interface__srv__CamPose_Request __attribute__((deprecated))
#else
# define DEPRECATED__interface__srv__CamPose_Request __declspec(deprecated)
#endif

namespace interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CamPose_Request_
{
  using Type = CamPose_Request_<ContainerAllocator>;

  explicit CamPose_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit CamPose_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    interface::srv::CamPose_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface::srv::CamPose_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface::srv::CamPose_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface::srv::CamPose_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface::srv::CamPose_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface::srv::CamPose_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface::srv::CamPose_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface::srv::CamPose_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface::srv::CamPose_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface::srv::CamPose_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface__srv__CamPose_Request
    std::shared_ptr<interface::srv::CamPose_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface__srv__CamPose_Request
    std::shared_ptr<interface::srv::CamPose_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CamPose_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const CamPose_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CamPose_Request_

// alias to use template instance with default allocator
using CamPose_Request =
  interface::srv::CamPose_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface


// Include directives for member types
// Member 'cam_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__interface__srv__CamPose_Response __attribute__((deprecated))
#else
# define DEPRECATED__interface__srv__CamPose_Response __declspec(deprecated)
#endif

namespace interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CamPose_Response_
{
  using Type = CamPose_Response_<ContainerAllocator>;

  explicit CamPose_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : cam_pose(_init)
  {
    (void)_init;
  }

  explicit CamPose_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : cam_pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _cam_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _cam_pose_type cam_pose;

  // setters for named parameter idiom
  Type & set__cam_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->cam_pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface::srv::CamPose_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface::srv::CamPose_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface::srv::CamPose_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface::srv::CamPose_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface::srv::CamPose_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface::srv::CamPose_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface::srv::CamPose_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface::srv::CamPose_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface::srv::CamPose_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface::srv::CamPose_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface__srv__CamPose_Response
    std::shared_ptr<interface::srv::CamPose_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface__srv__CamPose_Response
    std::shared_ptr<interface::srv::CamPose_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CamPose_Response_ & other) const
  {
    if (this->cam_pose != other.cam_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const CamPose_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CamPose_Response_

// alias to use template instance with default allocator
using CamPose_Response =
  interface::srv::CamPose_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface

namespace interface
{

namespace srv
{

struct CamPose
{
  using Request = interface::srv::CamPose_Request;
  using Response = interface::srv::CamPose_Response;
};

}  // namespace srv

}  // namespace interface

#endif  // INTERFACE__SRV__DETAIL__CAM_POSE__STRUCT_HPP_
