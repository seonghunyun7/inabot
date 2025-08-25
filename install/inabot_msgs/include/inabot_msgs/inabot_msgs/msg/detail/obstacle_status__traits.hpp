// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from inabot_msgs:msg/ObstacleStatus.idl
// generated code does not contain a copyright notice

#ifndef INABOT_MSGS__MSG__DETAIL__OBSTACLE_STATUS__TRAITS_HPP_
#define INABOT_MSGS__MSG__DETAIL__OBSTACLE_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "inabot_msgs/msg/detail/obstacle_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace inabot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ObstacleStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << ", ";
  }

  // member: stop
  {
    out << "stop: ";
    rosidl_generator_traits::value_to_yaml(msg.stop, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObstacleStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }

  // member: stop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stop: ";
    rosidl_generator_traits::value_to_yaml(msg.stop, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObstacleStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace inabot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use inabot_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const inabot_msgs::msg::ObstacleStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  inabot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inabot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const inabot_msgs::msg::ObstacleStatus & msg)
{
  return inabot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<inabot_msgs::msg::ObstacleStatus>()
{
  return "inabot_msgs::msg::ObstacleStatus";
}

template<>
inline const char * name<inabot_msgs::msg::ObstacleStatus>()
{
  return "inabot_msgs/msg/ObstacleStatus";
}

template<>
struct has_fixed_size<inabot_msgs::msg::ObstacleStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<inabot_msgs::msg::ObstacleStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<inabot_msgs::msg::ObstacleStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INABOT_MSGS__MSG__DETAIL__OBSTACLE_STATUS__TRAITS_HPP_
