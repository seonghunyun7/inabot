// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inabot_msgs:msg/ObstacleStatus.idl
// generated code does not contain a copyright notice

#ifndef INABOT_MSGS__MSG__DETAIL__OBSTACLE_STATUS__BUILDER_HPP_
#define INABOT_MSGS__MSG__DETAIL__OBSTACLE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inabot_msgs/msg/detail/obstacle_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inabot_msgs
{

namespace msg
{

namespace builder
{

class Init_ObstacleStatus_stop
{
public:
  explicit Init_ObstacleStatus_stop(::inabot_msgs::msg::ObstacleStatus & msg)
  : msg_(msg)
  {}
  ::inabot_msgs::msg::ObstacleStatus stop(::inabot_msgs::msg::ObstacleStatus::_stop_type arg)
  {
    msg_.stop = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inabot_msgs::msg::ObstacleStatus msg_;
};

class Init_ObstacleStatus_distance
{
public:
  Init_ObstacleStatus_distance()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObstacleStatus_stop distance(::inabot_msgs::msg::ObstacleStatus::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_ObstacleStatus_stop(msg_);
  }

private:
  ::inabot_msgs::msg::ObstacleStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inabot_msgs::msg::ObstacleStatus>()
{
  return inabot_msgs::msg::builder::Init_ObstacleStatus_distance();
}

}  // namespace inabot_msgs

#endif  // INABOT_MSGS__MSG__DETAIL__OBSTACLE_STATUS__BUILDER_HPP_
