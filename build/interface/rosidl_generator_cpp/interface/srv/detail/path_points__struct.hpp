// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface:srv/PathPoints.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__PATH_POINTS__STRUCT_HPP_
#define INTERFACE__SRV__DETAIL__PATH_POINTS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'start_position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'start_speed'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__interface__srv__PathPoints_Request __attribute__((deprecated))
#else
# define DEPRECATED__interface__srv__PathPoints_Request __declspec(deprecated)
#endif

namespace interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PathPoints_Request_
{
  using Type = PathPoints_Request_<ContainerAllocator>;

  explicit PathPoints_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start_position(_init),
    start_speed(_init)
  {
    (void)_init;
  }

  explicit PathPoints_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start_position(_alloc, _init),
    start_speed(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _start_position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _start_position_type start_position;
  using _start_speed_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _start_speed_type start_speed;

  // setters for named parameter idiom
  Type & set__start_position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->start_position = _arg;
    return *this;
  }
  Type & set__start_speed(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->start_speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface::srv::PathPoints_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface::srv::PathPoints_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface::srv::PathPoints_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface::srv::PathPoints_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface::srv::PathPoints_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface::srv::PathPoints_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface::srv::PathPoints_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface::srv::PathPoints_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface::srv::PathPoints_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface::srv::PathPoints_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface__srv__PathPoints_Request
    std::shared_ptr<interface::srv::PathPoints_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface__srv__PathPoints_Request
    std::shared_ptr<interface::srv::PathPoints_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PathPoints_Request_ & other) const
  {
    if (this->start_position != other.start_position) {
      return false;
    }
    if (this->start_speed != other.start_speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const PathPoints_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PathPoints_Request_

// alias to use template instance with default allocator
using PathPoints_Request =
  interface::srv::PathPoints_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface


// Include directives for member types
// Member 'path_points'
// already included above
// #include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__interface__srv__PathPoints_Response __attribute__((deprecated))
#else
# define DEPRECATED__interface__srv__PathPoints_Response __declspec(deprecated)
#endif

namespace interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PathPoints_Response_
{
  using Type = PathPoints_Response_<ContainerAllocator>;

  explicit PathPoints_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit PathPoints_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _path_points_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename ContainerAllocator::template rebind<geometry_msgs::msg::Point_<ContainerAllocator>>::other>;
  _path_points_type path_points;
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__path_points(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename ContainerAllocator::template rebind<geometry_msgs::msg::Point_<ContainerAllocator>>::other> & _arg)
  {
    this->path_points = _arg;
    return *this;
  }
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface::srv::PathPoints_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface::srv::PathPoints_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface::srv::PathPoints_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface::srv::PathPoints_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface::srv::PathPoints_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface::srv::PathPoints_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface::srv::PathPoints_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface::srv::PathPoints_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface::srv::PathPoints_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface::srv::PathPoints_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface__srv__PathPoints_Response
    std::shared_ptr<interface::srv::PathPoints_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface__srv__PathPoints_Response
    std::shared_ptr<interface::srv::PathPoints_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PathPoints_Response_ & other) const
  {
    if (this->path_points != other.path_points) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const PathPoints_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PathPoints_Response_

// alias to use template instance with default allocator
using PathPoints_Response =
  interface::srv::PathPoints_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface

namespace interface
{

namespace srv
{

struct PathPoints
{
  using Request = interface::srv::PathPoints_Request;
  using Response = interface::srv::PathPoints_Response;
};

}  // namespace srv

}  // namespace interface

#endif  // INTERFACE__SRV__DETAIL__PATH_POINTS__STRUCT_HPP_
