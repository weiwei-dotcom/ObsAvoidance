// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface:srv/Transform.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__TRANSFORM__STRUCT_HPP_
#define INTERFACE__SRV__DETAIL__TRANSFORM__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__interface__srv__Transform_Request __attribute__((deprecated))
#else
# define DEPRECATED__interface__srv__Transform_Request __declspec(deprecated)
#endif

namespace interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Transform_Request_
{
  using Type = Transform_Request_<ContainerAllocator>;

  explicit Transform_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit Transform_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    interface::srv::Transform_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface::srv::Transform_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface::srv::Transform_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface::srv::Transform_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface::srv::Transform_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface::srv::Transform_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface::srv::Transform_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface::srv::Transform_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface::srv::Transform_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface::srv::Transform_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface__srv__Transform_Request
    std::shared_ptr<interface::srv::Transform_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface__srv__Transform_Request
    std::shared_ptr<interface::srv::Transform_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Transform_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const Transform_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Transform_Request_

// alias to use template instance with default allocator
using Transform_Request =
  interface::srv::Transform_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface


// Include directives for member types
// Member 'transform_init_to_world'
// Member 'transform_base_to_world'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__interface__srv__Transform_Response __attribute__((deprecated))
#else
# define DEPRECATED__interface__srv__Transform_Response __declspec(deprecated)
#endif

namespace interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Transform_Response_
{
  using Type = Transform_Response_<ContainerAllocator>;

  explicit Transform_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : transform_init_to_world(_init),
    transform_base_to_world(_init)
  {
    (void)_init;
  }

  explicit Transform_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : transform_init_to_world(_alloc, _init),
    transform_base_to_world(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _transform_init_to_world_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _transform_init_to_world_type transform_init_to_world;
  using _transform_base_to_world_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _transform_base_to_world_type transform_base_to_world;

  // setters for named parameter idiom
  Type & set__transform_init_to_world(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->transform_init_to_world = _arg;
    return *this;
  }
  Type & set__transform_base_to_world(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->transform_base_to_world = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface::srv::Transform_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface::srv::Transform_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface::srv::Transform_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface::srv::Transform_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface::srv::Transform_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface::srv::Transform_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface::srv::Transform_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface::srv::Transform_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface::srv::Transform_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface::srv::Transform_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface__srv__Transform_Response
    std::shared_ptr<interface::srv::Transform_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface__srv__Transform_Response
    std::shared_ptr<interface::srv::Transform_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Transform_Response_ & other) const
  {
    if (this->transform_init_to_world != other.transform_init_to_world) {
      return false;
    }
    if (this->transform_base_to_world != other.transform_base_to_world) {
      return false;
    }
    return true;
  }
  bool operator!=(const Transform_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Transform_Response_

// alias to use template instance with default allocator
using Transform_Response =
  interface::srv::Transform_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface

namespace interface
{

namespace srv
{

struct Transform
{
  using Request = interface::srv::Transform_Request;
  using Response = interface::srv::Transform_Response;
};

}  // namespace srv

}  // namespace interface

#endif  // INTERFACE__SRV__DETAIL__TRANSFORM__STRUCT_HPP_
