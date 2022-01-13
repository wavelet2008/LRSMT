// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from webots_ros2_msgs:msg/WbCameraRecognitionObject.idl
// generated code does not contain a copyright notice

#ifndef WEBOTS_ROS2_MSGS__MSG__DETAIL__WB_CAMERA_RECOGNITION_OBJECT__FUNCTIONS_H_
#define WEBOTS_ROS2_MSGS__MSG__DETAIL__WB_CAMERA_RECOGNITION_OBJECT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "webots_ros2_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "webots_ros2_msgs/msg/detail/wb_camera_recognition_object__struct.h"

/// Initialize msg/WbCameraRecognitionObject message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * webots_ros2_msgs__msg__WbCameraRecognitionObject
 * )) before or use
 * webots_ros2_msgs__msg__WbCameraRecognitionObject__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_webots_ros2_msgs
bool
webots_ros2_msgs__msg__WbCameraRecognitionObject__init(webots_ros2_msgs__msg__WbCameraRecognitionObject * msg);

/// Finalize msg/WbCameraRecognitionObject message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_webots_ros2_msgs
void
webots_ros2_msgs__msg__WbCameraRecognitionObject__fini(webots_ros2_msgs__msg__WbCameraRecognitionObject * msg);

/// Create msg/WbCameraRecognitionObject message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * webots_ros2_msgs__msg__WbCameraRecognitionObject__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_webots_ros2_msgs
webots_ros2_msgs__msg__WbCameraRecognitionObject *
webots_ros2_msgs__msg__WbCameraRecognitionObject__create();

/// Destroy msg/WbCameraRecognitionObject message.
/**
 * It calls
 * webots_ros2_msgs__msg__WbCameraRecognitionObject__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_webots_ros2_msgs
void
webots_ros2_msgs__msg__WbCameraRecognitionObject__destroy(webots_ros2_msgs__msg__WbCameraRecognitionObject * msg);


/// Initialize array of msg/WbCameraRecognitionObject messages.
/**
 * It allocates the memory for the number of elements and calls
 * webots_ros2_msgs__msg__WbCameraRecognitionObject__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_webots_ros2_msgs
bool
webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence__init(webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence * array, size_t size);

/// Finalize array of msg/WbCameraRecognitionObject messages.
/**
 * It calls
 * webots_ros2_msgs__msg__WbCameraRecognitionObject__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_webots_ros2_msgs
void
webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence__fini(webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence * array);

/// Create array of msg/WbCameraRecognitionObject messages.
/**
 * It allocates the memory for the array and calls
 * webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_webots_ros2_msgs
webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence *
webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence__create(size_t size);

/// Destroy array of msg/WbCameraRecognitionObject messages.
/**
 * It calls
 * webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_webots_ros2_msgs
void
webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence__destroy(webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // WEBOTS_ROS2_MSGS__MSG__DETAIL__WB_CAMERA_RECOGNITION_OBJECT__FUNCTIONS_H_
