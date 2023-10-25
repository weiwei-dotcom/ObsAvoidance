// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface:srv/T.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__T__STRUCT_HPP_
#define INTERFACE__SRV__DETAIL__T__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__interface__srv__T_Request __attribute__((deprecated))
#else
# define DEPRECATED__interface__srv__T_Request __declspec(deprecated)
#endif

namespace interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct T_Request_
{
  using Type = T_Request_<ContainerAllocator>;

  explicit T_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request_flag = 0l;
    }
  }

  explicit T_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request_flag = 0l;
    }
  }

  // field types and members
  using _request_flag_type =
    int32_t;
  _request_flag_type request_flag;

  // setters for named parameter idiom
  Type & set__request_flag(
    const int32_t & _arg)
  {
    this->request_flag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface::srv::T_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface::srv::T_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface::srv::T_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface::srv::T_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface::srv::T_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface::srv::T_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface::srv::T_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface::srv::T_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface::srv::T_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface::srv::T_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface__srv__T_Request
    std::shared_ptr<interface::srv::T_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface__srv__T_Request
    std::shared_ptr<interface::srv::T_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const T_Request_ & other) const
  {
    if (this->request_flag != other.request_flag) {
      return false;
    }
    return true;
  }
  bool operator!=(const T_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct T_Request_

// alias to use template instance with default allocator
using T_Request =
  interface::srv::T_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface


// Include directives for member types
// Member 'transform_init_to_world'
// Member 'transform_base_to_world'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__interface__srv__T_Response __attribute__((deprecated))
#else
# define DEPRECATED__interface__srv__T_Response __declspec(deprecated)
#endif

namespace interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct T_Response_
{
  using Type = T_Response_<ContainerAllocator>;

  explicit T_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : transform_init_to_world(_init),
    transform_base_to_world(_init)
  {
    (void)_init;
  }

  explicit T_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    interface::srv::T_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface::srv::T_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface::srv::T_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface::srv::T_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface::srv::T_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface::srv::T_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface::srv::T_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface::srv::T_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface::srv::T_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface::srv::T_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface__srv__T_Response
    std::shared_ptr<interface::srv::T_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface__srv__T_Response
    std::shared_ptr<interface::srv::T_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const T_Response_ & other) const
  {
    if (this->transform_init_to_world != other.transform_init_to_world) {
      return false;
    }
    if (this->transform_base_to_world != other.transform_base_to_world) {
      return false;
    }
    return true;
  }
  bool operator!=(const T_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct T_Response_

// alias to use template instance with default allocator
using T_Response =
  interface::srv::T_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface

namespace interface
{

namespace srv
{

struct T
{
  using Request = interface::srv::T_Request;
  using Response = interface::srv::T_Response;
};

}  // namespace srv

}  // namespace interface

#endif  // INTERFACE__SRV__DETAIL__T__STRUCT_HPP_
