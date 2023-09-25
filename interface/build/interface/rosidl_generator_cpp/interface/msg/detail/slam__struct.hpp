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
// Member 'point_cloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"
// Member 'cam_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'keypoints'
#include "geometry_msgs/msg/detail/point32__struct.hpp"

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
  : point_cloud(_init),
    cam_pose(_init)
  {
    (void)_init;
  }

  explicit Slam_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : point_cloud(_alloc, _init),
    cam_pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _point_cloud_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _point_cloud_type point_cloud;
  using _cam_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _cam_pose_type cam_pose;
  using _keypoints_type =
    std::vector<geometry_msgs::msg::Point32_<ContainerAllocator>, typename ContainerAllocator::template rebind<geometry_msgs::msg::Point32_<ContainerAllocator>>::other>;
  _keypoints_type keypoints;

  // setters for named parameter idiom
  Type & set__point_cloud(
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> & _arg)
  {
    this->point_cloud = _arg;
    return *this;
  }
  Type & set__cam_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->cam_pose = _arg;
    return *this;
  }
  Type & set__keypoints(
    const std::vector<geometry_msgs::msg::Point32_<ContainerAllocator>, typename ContainerAllocator::template rebind<geometry_msgs::msg::Point32_<ContainerAllocator>>::other> & _arg)
  {
    this->keypoints = _arg;
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
    if (this->point_cloud != other.point_cloud) {
      return false;
    }
    if (this->cam_pose != other.cam_pose) {
      return false;
    }
    if (this->keypoints != other.keypoints) {
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
