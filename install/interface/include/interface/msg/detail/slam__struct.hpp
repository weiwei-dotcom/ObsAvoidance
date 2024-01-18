// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface:msg/Slam.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__MSG__DETAIL__SLAM__STRUCT_HPP_
#define INTERFACE__MSG__DETAIL__SLAM__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'point_cloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"
// Member 'transform_init2cur'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__interface__msg__Slam __attribute__((deprecated))
#else
# define DEPRECATED__interface__msg__Slam __declspec(deprecated)
#endif

namespace interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Slam_
{
  using Type = Slam_<ContainerAllocator>;

  explicit Slam_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    point_cloud(_init),
    transform_init2cur(_init)
  {
    (void)_init;
  }

  explicit Slam_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    point_cloud(_alloc, _init),
    transform_init2cur(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _point_cloud_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _point_cloud_type point_cloud;
  using _transform_init2cur_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _transform_init2cur_type transform_init2cur;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__point_cloud(
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> & _arg)
  {
    this->point_cloud = _arg;
    return *this;
  }
  Type & set__transform_init2cur(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->transform_init2cur = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface::msg::Slam_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface::msg::Slam_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface::msg::Slam_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface::msg::Slam_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface::msg::Slam_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface::msg::Slam_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface::msg::Slam_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface::msg::Slam_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface::msg::Slam_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface::msg::Slam_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface__msg__Slam
    std::shared_ptr<interface::msg::Slam_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface__msg__Slam
    std::shared_ptr<interface::msg::Slam_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Slam_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->point_cloud != other.point_cloud) {
      return false;
    }
    if (this->transform_init2cur != other.transform_init2cur) {
      return false;
    }
    return true;
  }
  bool operator!=(const Slam_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Slam_

// alias to use template instance with default allocator
using Slam =
  interface::msg::Slam_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interface

#endif  // INTERFACE__MSG__DETAIL__SLAM__STRUCT_HPP_
