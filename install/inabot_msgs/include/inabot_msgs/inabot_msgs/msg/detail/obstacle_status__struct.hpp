// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inabot_msgs:msg/ObstacleStatus.idl
// generated code does not contain a copyright notice

#ifndef INABOT_MSGS__MSG__DETAIL__OBSTACLE_STATUS__STRUCT_HPP_
#define INABOT_MSGS__MSG__DETAIL__OBSTACLE_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__inabot_msgs__msg__ObstacleStatus __attribute__((deprecated))
#else
# define DEPRECATED__inabot_msgs__msg__ObstacleStatus __declspec(deprecated)
#endif

namespace inabot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObstacleStatus_
{
  using Type = ObstacleStatus_<ContainerAllocator>;

  explicit ObstacleStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->distance = 0.0;
      this->stop = false;
    }
  }

  explicit ObstacleStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->distance = 0.0;
      this->stop = false;
    }
  }

  // field types and members
  using _distance_type =
    double;
  _distance_type distance;
  using _stop_type =
    bool;
  _stop_type stop;

  // setters for named parameter idiom
  Type & set__distance(
    const double & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__stop(
    const bool & _arg)
  {
    this->stop = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inabot_msgs::msg::ObstacleStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const inabot_msgs::msg::ObstacleStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inabot_msgs::msg::ObstacleStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inabot_msgs::msg::ObstacleStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inabot_msgs::msg::ObstacleStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inabot_msgs::msg::ObstacleStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inabot_msgs::msg::ObstacleStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inabot_msgs::msg::ObstacleStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inabot_msgs::msg::ObstacleStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inabot_msgs::msg::ObstacleStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inabot_msgs__msg__ObstacleStatus
    std::shared_ptr<inabot_msgs::msg::ObstacleStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inabot_msgs__msg__ObstacleStatus
    std::shared_ptr<inabot_msgs::msg::ObstacleStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObstacleStatus_ & other) const
  {
    if (this->distance != other.distance) {
      return false;
    }
    if (this->stop != other.stop) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObstacleStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObstacleStatus_

// alias to use template instance with default allocator
using ObstacleStatus =
  inabot_msgs::msg::ObstacleStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace inabot_msgs

#endif  // INABOT_MSGS__MSG__DETAIL__OBSTACLE_STATUS__STRUCT_HPP_
