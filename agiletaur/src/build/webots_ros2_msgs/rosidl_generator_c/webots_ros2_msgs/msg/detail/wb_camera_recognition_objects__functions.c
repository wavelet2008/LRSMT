// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from webots_ros2_msgs:msg/WbCameraRecognitionObjects.idl
// generated code does not contain a copyright notice
#include "webots_ros2_msgs/msg/detail/wb_camera_recognition_objects__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `objects`
#include "webots_ros2_msgs/msg/detail/wb_camera_recognition_object__functions.h"

bool
webots_ros2_msgs__msg__WbCameraRecognitionObjects__init(webots_ros2_msgs__msg__WbCameraRecognitionObjects * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    webots_ros2_msgs__msg__WbCameraRecognitionObjects__fini(msg);
    return false;
  }
  // objects
  if (!webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence__init(&msg->objects, 0)) {
    webots_ros2_msgs__msg__WbCameraRecognitionObjects__fini(msg);
    return false;
  }
  return true;
}

void
webots_ros2_msgs__msg__WbCameraRecognitionObjects__fini(webots_ros2_msgs__msg__WbCameraRecognitionObjects * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // objects
  webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence__fini(&msg->objects);
}

webots_ros2_msgs__msg__WbCameraRecognitionObjects *
webots_ros2_msgs__msg__WbCameraRecognitionObjects__create()
{
  webots_ros2_msgs__msg__WbCameraRecognitionObjects * msg = (webots_ros2_msgs__msg__WbCameraRecognitionObjects *)malloc(sizeof(webots_ros2_msgs__msg__WbCameraRecognitionObjects));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(webots_ros2_msgs__msg__WbCameraRecognitionObjects));
  bool success = webots_ros2_msgs__msg__WbCameraRecognitionObjects__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
webots_ros2_msgs__msg__WbCameraRecognitionObjects__destroy(webots_ros2_msgs__msg__WbCameraRecognitionObjects * msg)
{
  if (msg) {
    webots_ros2_msgs__msg__WbCameraRecognitionObjects__fini(msg);
  }
  free(msg);
}


bool
webots_ros2_msgs__msg__WbCameraRecognitionObjects__Sequence__init(webots_ros2_msgs__msg__WbCameraRecognitionObjects__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  webots_ros2_msgs__msg__WbCameraRecognitionObjects * data = NULL;
  if (size) {
    data = (webots_ros2_msgs__msg__WbCameraRecognitionObjects *)calloc(size, sizeof(webots_ros2_msgs__msg__WbCameraRecognitionObjects));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = webots_ros2_msgs__msg__WbCameraRecognitionObjects__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        webots_ros2_msgs__msg__WbCameraRecognitionObjects__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
webots_ros2_msgs__msg__WbCameraRecognitionObjects__Sequence__fini(webots_ros2_msgs__msg__WbCameraRecognitionObjects__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      webots_ros2_msgs__msg__WbCameraRecognitionObjects__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

webots_ros2_msgs__msg__WbCameraRecognitionObjects__Sequence *
webots_ros2_msgs__msg__WbCameraRecognitionObjects__Sequence__create(size_t size)
{
  webots_ros2_msgs__msg__WbCameraRecognitionObjects__Sequence * array = (webots_ros2_msgs__msg__WbCameraRecognitionObjects__Sequence *)malloc(sizeof(webots_ros2_msgs__msg__WbCameraRecognitionObjects__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = webots_ros2_msgs__msg__WbCameraRecognitionObjects__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
webots_ros2_msgs__msg__WbCameraRecognitionObjects__Sequence__destroy(webots_ros2_msgs__msg__WbCameraRecognitionObjects__Sequence * array)
{
  if (array) {
    webots_ros2_msgs__msg__WbCameraRecognitionObjects__Sequence__fini(array);
  }
  free(array);
}
