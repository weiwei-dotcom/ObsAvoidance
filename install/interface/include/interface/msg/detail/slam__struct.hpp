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
<<<<<<< HEAD
// Member 'point_cloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"
// Member 'cam_pose'
// Member 'world2cam'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'img'
#include "sensor_msgs/msg/detail/image__struct.hpp"
=======
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'point_cloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"
// Member 'transform_init2cur'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09

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
<<<<<<< HEAD
  : point_cloud(_init),
    cam_pose(_init),
    world2cam(_init),
    img(_init)
=======
  : header(_init),
    point_cloud(_init),
    transform_init2cur(_init)
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  {
    (void)_init;
  }

  explicit Slam_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
<<<<<<< HEAD
  : point_cloud(_alloc, _init),
    cam_pose(_alloc, _init),
    world2cam(_alloc, _init),
    img(_alloc, _init)
=======
  : header(_alloc, _init),
    point_cloud(_alloc, _init),
    transform_init2cur(_alloc, _init)
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  {
    (void)_init;
  }

  // field types and members
<<<<<<< HEAD
  using _point_cloud_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _point_cloud_type point_cloud;
  using _cam_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _cam_pose_type cam_pose;
  using _world2cam_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _world2cam_type world2cam;
  using _img_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _img_type img;

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
  Type & set__world2cam(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->world2cam = _arg;
    return *this;
  }
  Type & set__img(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->img = _arg;
=======
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
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
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
<<<<<<< HEAD
    if (this->point_cloud != other.point_cloud) {
      return false;
    }
    if (this->cam_pose != other.cam_pose) {
      return false;
    }
    if (this->world2cam != other.world2cam) {
      return false;
    }
    if (this->img != other.img) {
=======
    if (this->header != other.header) {
      return false;
    }
    if (this->point_cloud != other.point_cloud) {
      return false;
    }
    if (this->transform_init2cur != other.transform_init2cur) {
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
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
