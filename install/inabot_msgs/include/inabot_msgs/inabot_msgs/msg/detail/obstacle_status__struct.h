// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inabot_msgs:msg/ObstacleStatus.idl
// generated code does not contain a copyright notice

#ifndef INABOT_MSGS__MSG__DETAIL__OBSTACLE_STATUS__STRUCT_H_
#define INABOT_MSGS__MSG__DETAIL__OBSTACLE_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ObstacleStatus in the package inabot_msgs.
/**
  * 전방 장애물 상태 메시지
 */
typedef struct inabot_msgs__msg__ObstacleStatus
{
  /// 전방 장애물까지 거리 (m)
  double distance;
  /// 장애물 정지 플래그
  bool stop;
} inabot_msgs__msg__ObstacleStatus;

// Struct for a sequence of inabot_msgs__msg__ObstacleStatus.
typedef struct inabot_msgs__msg__ObstacleStatus__Sequence
{
  inabot_msgs__msg__ObstacleStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inabot_msgs__msg__ObstacleStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INABOT_MSGS__MSG__DETAIL__OBSTACLE_STATUS__STRUCT_H_
