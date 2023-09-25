// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface:srv/MapPoint.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__MAP_POINT__STRUCT_HPP_
#define INTERFACE__SRV__DETAIL__MAP_POINT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__interface__srv__MapPoint_Request __attribute__((deprecated))
#else
# define DEPRECATED__interface__srv__MapPoint_Request __declspec(deprecated)
#endif

namespace interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MapPoint_Request_
{
  using Type = MapPoint_Request_<ContainerAllocator>;

  explicit MapPoint_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit MapPoint_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    interface::srv::MapPoint_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface::srv::MapPoint_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface::srv::MapPoint_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface::srv::MapPoint_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface::srv::MapPoint_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface::srv::MapPoint_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface::srv::MapPoint_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface::srv::MapPoint_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface::srv::MapPoint_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface::srv::MapPoint_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface__srv__MapPoint_Request
    std::shared_ptr<interface::srv::MapPoint_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface__srv__MapPoint_Request
    std::shared_ptr<interface::srv::MapPoint_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MapPoint_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const MapPoint_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MapPoint_Request_

// alias to use template instance with default allocator
using MapPoint_Request =
  interface::srv::MapPoint_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface


// Include directives for member types
// Member 'point_cloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"
// Member 'img'
#include "sensor_msgs/msg/detail/image__struct.hpp"
// Member 'cam_pose'
// Member 'world2cam'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__interface__srv__MapPoint_Response __attribute__((deprecated))
#else
# define DEPRECATED__interface__srv__MapPoint_Response __declspec(deprecated)
#endif

namespace interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MapPoint_Response_
{
  using Type = MapPoint_Response_<ContainerAllocator>;

  explicit MapPoint_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : point_cloud(_init),
    img(_init),
    cam_pose(_init),
    world2cam(_init)
  {
    (void)_init;
  }

  explicit MapPoint_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : point_cloud(_alloc, _init),
    img(_alloc, _init),
    cam_pose(_alloc, _init),
    world2cam(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _point_cloud_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _point_cloud_type point_cloud;
  using _img_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _img_type img;
  using _cam_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _cam_pose_type cam_pose;
  using _world2cam_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _world2cam_type world2cam;

  // setters for named parameter idiom
  Type & set__point_cloud(
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> & _arg)
  {
    this->point_cloud = _arg;
    return *this;
  }
  Type & set__img(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->img = _arg;
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

  // constant declarations

  // pointer types
  using RawPtr =
    interface::srv::MapPoint_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface::srv::MapPoint_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface::srv::MapPoint_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface::srv::MapPoint_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface::srv::MapPoint_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface::srv::MapPoint_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface::srv::MapPoint_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface::srv::MapPoint_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface::srv::MapPoint_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface::srv::MapPoint_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface__srv__MapPoint_Response
    std::shared_ptr<interface::srv::MapPoint_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface__srv__MapPoint_Response
    std::shared_ptr<interface::srv::MapPoint_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MapPoint_Response_ & other) const
  {
    if (this->point_cloud != other.point_cloud) {
      return false;
    }
    if (this->img != other.img) {
      return false;
    }
    if (this->cam_pose != other.cam_pose) {
      return false;
    }
    if (this->world2cam != other.world2cam) {
      return false;
    }
    return true;
  }
  bool operator!=(const MapPoint_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MapPoint_Response_

// alias to use template instance with default allocator
using MapPoint_Response =
  interface::srv::MapPoint_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface

namespace interface
{

namespace srv
{

struct MapPoint
{
  using Request = interface::srv::MapPoint_Request;
  using Response = interface::srv::MapPoint_Response;
};

}  // namespace srv

}  // namespace interface

#endif  // INTERFACE__SRV__DETAIL__MAP_POINT__STRUCT_HPP_
