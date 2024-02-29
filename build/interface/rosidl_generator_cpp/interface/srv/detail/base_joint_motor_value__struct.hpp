// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface:srv/BaseJointMotorValue.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__BASE_JOINT_MOTOR_VALUE__STRUCT_HPP_
#define INTERFACE__SRV__DETAIL__BASE_JOINT_MOTOR_VALUE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__interface__srv__BaseJointMotorValue_Request __attribute__((deprecated))
#else
# define DEPRECATED__interface__srv__BaseJointMotorValue_Request __declspec(deprecated)
#endif

namespace interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct BaseJointMotorValue_Request_
{
  using Type = BaseJointMotorValue_Request_<ContainerAllocator>;

  explicit BaseJointMotorValue_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit BaseJointMotorValue_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    interface::srv::BaseJointMotorValue_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface::srv::BaseJointMotorValue_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface::srv::BaseJointMotorValue_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface::srv::BaseJointMotorValue_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface::srv::BaseJointMotorValue_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface::srv::BaseJointMotorValue_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface::srv::BaseJointMotorValue_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface::srv::BaseJointMotorValue_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface::srv::BaseJointMotorValue_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface::srv::BaseJointMotorValue_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface__srv__BaseJointMotorValue_Request
    std::shared_ptr<interface::srv::BaseJointMotorValue_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface__srv__BaseJointMotorValue_Request
    std::shared_ptr<interface::srv::BaseJointMotorValue_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BaseJointMotorValue_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const BaseJointMotorValue_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BaseJointMotorValue_Request_

// alias to use template instance with default allocator
using BaseJointMotorValue_Request =
  interface::srv::BaseJointMotorValue_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface


// Include directives for member types
// Member 'cable_expend_value'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__interface__srv__BaseJointMotorValue_Response __attribute__((deprecated))
#else
# define DEPRECATED__interface__srv__BaseJointMotorValue_Response __declspec(deprecated)
#endif

namespace interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct BaseJointMotorValue_Response_
{
  using Type = BaseJointMotorValue_Response_<ContainerAllocator>;

  explicit BaseJointMotorValue_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->base_advance_value = 0.0;
      this->flag_base_out_range = false;
      this->flag_arrived_target = false;
    }
  }

  explicit BaseJointMotorValue_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->base_advance_value = 0.0;
      this->flag_base_out_range = false;
      this->flag_arrived_target = false;
    }
  }

  // field types and members
  using _cable_expend_value_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename ContainerAllocator::template rebind<geometry_msgs::msg::Point_<ContainerAllocator>>::other>;
  _cable_expend_value_type cable_expend_value;
  using _base_advance_value_type =
    double;
  _base_advance_value_type base_advance_value;
  using _flag_base_out_range_type =
    bool;
  _flag_base_out_range_type flag_base_out_range;
  using _flag_arrived_target_type =
    bool;
  _flag_arrived_target_type flag_arrived_target;

  // setters for named parameter idiom
  Type & set__cable_expend_value(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename ContainerAllocator::template rebind<geometry_msgs::msg::Point_<ContainerAllocator>>::other> & _arg)
  {
    this->cable_expend_value = _arg;
    return *this;
  }
  Type & set__base_advance_value(
    const double & _arg)
  {
    this->base_advance_value = _arg;
    return *this;
  }
  Type & set__flag_base_out_range(
    const bool & _arg)
  {
    this->flag_base_out_range = _arg;
    return *this;
  }
  Type & set__flag_arrived_target(
    const bool & _arg)
  {
    this->flag_arrived_target = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface::srv::BaseJointMotorValue_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface::srv::BaseJointMotorValue_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface::srv::BaseJointMotorValue_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface::srv::BaseJointMotorValue_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface::srv::BaseJointMotorValue_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface::srv::BaseJointMotorValue_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface::srv::BaseJointMotorValue_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface::srv::BaseJointMotorValue_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface::srv::BaseJointMotorValue_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface::srv::BaseJointMotorValue_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface__srv__BaseJointMotorValue_Response
    std::shared_ptr<interface::srv::BaseJointMotorValue_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface__srv__BaseJointMotorValue_Response
    std::shared_ptr<interface::srv::BaseJointMotorValue_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BaseJointMotorValue_Response_ & other) const
  {
    if (this->cable_expend_value != other.cable_expend_value) {
      return false;
    }
    if (this->base_advance_value != other.base_advance_value) {
      return false;
    }
    if (this->flag_base_out_range != other.flag_base_out_range) {
      return false;
    }
    if (this->flag_arrived_target != other.flag_arrived_target) {
      return false;
    }
    return true;
  }
  bool operator!=(const BaseJointMotorValue_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BaseJointMotorValue_Response_

// alias to use template instance with default allocator
using BaseJointMotorValue_Response =
  interface::srv::BaseJointMotorValue_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface

namespace interface
{

namespace srv
{

struct BaseJointMotorValue
{
  using Request = interface::srv::BaseJointMotorValue_Request;
  using Response = interface::srv::BaseJointMotorValue_Response;
};

}  // namespace srv

}  // namespace interface

#endif  // INTERFACE__SRV__DETAIL__BASE_JOINT_MOTOR_VALUE__STRUCT_HPP_
