// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from webots_ros2_msgs:srv/SetInt.idl
// generated code does not contain a copyright notice

#ifndef WEBOTS_ROS2_MSGS__SRV__DETAIL__SET_INT__STRUCT_H_
#define WEBOTS_ROS2_MSGS__SRV__DETAIL__SET_INT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/SetInt in the package webots_ros2_msgs.
typedef struct webots_ros2_msgs__srv__SetInt_Request
{
  int32_t value;
} webots_ros2_msgs__srv__SetInt_Request;

// Struct for a sequence of webots_ros2_msgs__srv__SetInt_Request.
typedef struct webots_ros2_msgs__srv__SetInt_Request__Sequence
{
  webots_ros2_msgs__srv__SetInt_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} webots_ros2_msgs__srv__SetInt_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/SetInt in the package webots_ros2_msgs.
typedef struct webots_ros2_msgs__srv__SetInt_Response
{
  bool success;
} webots_ros2_msgs__srv__SetInt_Response;

// Struct for a sequence of webots_ros2_msgs__srv__SetInt_Response.
typedef struct webots_ros2_msgs__srv__SetInt_Response__Sequence
{
  webots_ros2_msgs__srv__SetInt_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} webots_ros2_msgs__srv__SetInt_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WEBOTS_ROS2_MSGS__SRV__DETAIL__SET_INT__STRUCT_H_
