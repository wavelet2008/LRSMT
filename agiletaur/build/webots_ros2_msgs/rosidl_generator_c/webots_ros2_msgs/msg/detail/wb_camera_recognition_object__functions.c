// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from webots_ros2_msgs:msg/WbCameraRecognitionObject.idl
// generated code does not contain a copyright notice
#include "webots_ros2_msgs/msg/detail/wb_camera_recognition_object__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
// Member `bbox`
#include "vision_msgs/msg/detail/bounding_box2_d__functions.h"
// Member `colors`
#include "std_msgs/msg/detail/color_rgba__functions.h"
// Member `model`
#include "rosidl_runtime_c/string_functions.h"

bool
webots_ros2_msgs__msg__WbCameraRecognitionObject__init(webots_ros2_msgs__msg__WbCameraRecognitionObject * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->pose)) {
    webots_ros2_msgs__msg__WbCameraRecognitionObject__fini(msg);
    return false;
  }
  // bbox
  if (!vision_msgs__msg__BoundingBox2D__init(&msg->bbox)) {
    webots_ros2_msgs__msg__WbCameraRecognitionObject__fini(msg);
    return false;
  }
  // colors
  if (!std_msgs__msg__ColorRGBA__Sequence__init(&msg->colors, 0)) {
    webots_ros2_msgs__msg__WbCameraRecognitionObject__fini(msg);
    return false;
  }
  // model
  if (!rosidl_runtime_c__String__init(&msg->model)) {
    webots_ros2_msgs__msg__WbCameraRecognitionObject__fini(msg);
    return false;
  }
  return true;
}

void
webots_ros2_msgs__msg__WbCameraRecognitionObject__fini(webots_ros2_msgs__msg__WbCameraRecognitionObject * msg)
{
  if (!msg) {
    return;
  }
  // id
  // pose
  geometry_msgs__msg__PoseStamped__fini(&msg->pose);
  // bbox
  vision_msgs__msg__BoundingBox2D__fini(&msg->bbox);
  // colors
  std_msgs__msg__ColorRGBA__Sequence__fini(&msg->colors);
  // model
  rosidl_runtime_c__String__fini(&msg->model);
}

webots_ros2_msgs__msg__WbCameraRecognitionObject *
webots_ros2_msgs__msg__WbCameraRecognitionObject__create()
{
  webots_ros2_msgs__msg__WbCameraRecognitionObject * msg = (webots_ros2_msgs__msg__WbCameraRecognitionObject *)malloc(sizeof(webots_ros2_msgs__msg__WbCameraRecognitionObject));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(webots_ros2_msgs__msg__WbCameraRecognitionObject));
  bool success = webots_ros2_msgs__msg__WbCameraRecognitionObject__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
webots_ros2_msgs__msg__WbCameraRecognitionObject__destroy(webots_ros2_msgs__msg__WbCameraRecognitionObject * msg)
{
  if (msg) {
    webots_ros2_msgs__msg__WbCameraRecognitionObject__fini(msg);
  }
  free(msg);
}


bool
webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence__init(webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  webots_ros2_msgs__msg__WbCameraRecognitionObject * data = NULL;
  if (size) {
    data = (webots_ros2_msgs__msg__WbCameraRecognitionObject *)calloc(size, sizeof(webots_ros2_msgs__msg__WbCameraRecognitionObject));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = webots_ros2_msgs__msg__WbCameraRecognitionObject__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        webots_ros2_msgs__msg__WbCameraRecognitionObject__fini(&data[i - 1]);
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
webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence__fini(webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      webots_ros2_msgs__msg__WbCameraRecognitionObject__fini(&array->data[i]);
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

webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence *
webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence__create(size_t size)
{
  webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence * array = (webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence *)malloc(sizeof(webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence__destroy(webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence * array)
{
  if (array) {
    webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence__fini(array);
  }
  free(array);
}
