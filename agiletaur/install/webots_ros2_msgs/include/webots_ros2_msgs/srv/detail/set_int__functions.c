// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from webots_ros2_msgs:srv/SetInt.idl
// generated code does not contain a copyright notice
#include "webots_ros2_msgs/srv/detail/set_int__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

bool
webots_ros2_msgs__srv__SetInt_Request__init(webots_ros2_msgs__srv__SetInt_Request * msg)
{
  if (!msg) {
    return false;
  }
  // value
  return true;
}

void
webots_ros2_msgs__srv__SetInt_Request__fini(webots_ros2_msgs__srv__SetInt_Request * msg)
{
  if (!msg) {
    return;
  }
  // value
}

webots_ros2_msgs__srv__SetInt_Request *
webots_ros2_msgs__srv__SetInt_Request__create()
{
  webots_ros2_msgs__srv__SetInt_Request * msg = (webots_ros2_msgs__srv__SetInt_Request *)malloc(sizeof(webots_ros2_msgs__srv__SetInt_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(webots_ros2_msgs__srv__SetInt_Request));
  bool success = webots_ros2_msgs__srv__SetInt_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
webots_ros2_msgs__srv__SetInt_Request__destroy(webots_ros2_msgs__srv__SetInt_Request * msg)
{
  if (msg) {
    webots_ros2_msgs__srv__SetInt_Request__fini(msg);
  }
  free(msg);
}


bool
webots_ros2_msgs__srv__SetInt_Request__Sequence__init(webots_ros2_msgs__srv__SetInt_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  webots_ros2_msgs__srv__SetInt_Request * data = NULL;
  if (size) {
    data = (webots_ros2_msgs__srv__SetInt_Request *)calloc(size, sizeof(webots_ros2_msgs__srv__SetInt_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = webots_ros2_msgs__srv__SetInt_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        webots_ros2_msgs__srv__SetInt_Request__fini(&data[i - 1]);
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
webots_ros2_msgs__srv__SetInt_Request__Sequence__fini(webots_ros2_msgs__srv__SetInt_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      webots_ros2_msgs__srv__SetInt_Request__fini(&array->data[i]);
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

webots_ros2_msgs__srv__SetInt_Request__Sequence *
webots_ros2_msgs__srv__SetInt_Request__Sequence__create(size_t size)
{
  webots_ros2_msgs__srv__SetInt_Request__Sequence * array = (webots_ros2_msgs__srv__SetInt_Request__Sequence *)malloc(sizeof(webots_ros2_msgs__srv__SetInt_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = webots_ros2_msgs__srv__SetInt_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
webots_ros2_msgs__srv__SetInt_Request__Sequence__destroy(webots_ros2_msgs__srv__SetInt_Request__Sequence * array)
{
  if (array) {
    webots_ros2_msgs__srv__SetInt_Request__Sequence__fini(array);
  }
  free(array);
}


bool
webots_ros2_msgs__srv__SetInt_Response__init(webots_ros2_msgs__srv__SetInt_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
webots_ros2_msgs__srv__SetInt_Response__fini(webots_ros2_msgs__srv__SetInt_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

webots_ros2_msgs__srv__SetInt_Response *
webots_ros2_msgs__srv__SetInt_Response__create()
{
  webots_ros2_msgs__srv__SetInt_Response * msg = (webots_ros2_msgs__srv__SetInt_Response *)malloc(sizeof(webots_ros2_msgs__srv__SetInt_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(webots_ros2_msgs__srv__SetInt_Response));
  bool success = webots_ros2_msgs__srv__SetInt_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
webots_ros2_msgs__srv__SetInt_Response__destroy(webots_ros2_msgs__srv__SetInt_Response * msg)
{
  if (msg) {
    webots_ros2_msgs__srv__SetInt_Response__fini(msg);
  }
  free(msg);
}


bool
webots_ros2_msgs__srv__SetInt_Response__Sequence__init(webots_ros2_msgs__srv__SetInt_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  webots_ros2_msgs__srv__SetInt_Response * data = NULL;
  if (size) {
    data = (webots_ros2_msgs__srv__SetInt_Response *)calloc(size, sizeof(webots_ros2_msgs__srv__SetInt_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = webots_ros2_msgs__srv__SetInt_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        webots_ros2_msgs__srv__SetInt_Response__fini(&data[i - 1]);
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
webots_ros2_msgs__srv__SetInt_Response__Sequence__fini(webots_ros2_msgs__srv__SetInt_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      webots_ros2_msgs__srv__SetInt_Response__fini(&array->data[i]);
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

webots_ros2_msgs__srv__SetInt_Response__Sequence *
webots_ros2_msgs__srv__SetInt_Response__Sequence__create(size_t size)
{
  webots_ros2_msgs__srv__SetInt_Response__Sequence * array = (webots_ros2_msgs__srv__SetInt_Response__Sequence *)malloc(sizeof(webots_ros2_msgs__srv__SetInt_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = webots_ros2_msgs__srv__SetInt_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
webots_ros2_msgs__srv__SetInt_Response__Sequence__destroy(webots_ros2_msgs__srv__SetInt_Response__Sequence * array)
{
  if (array) {
    webots_ros2_msgs__srv__SetInt_Response__Sequence__fini(array);
  }
  free(array);
}
