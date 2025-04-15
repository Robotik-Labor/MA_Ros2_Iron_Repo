// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from common_services_package:srv/GetPointCloud2.idl
// generated code does not contain a copyright notice
#include "common_services_package/srv/detail/get_point_cloud2__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
common_services_package__srv__GetPointCloud2_Request__init(common_services_package__srv__GetPointCloud2_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
common_services_package__srv__GetPointCloud2_Request__fini(common_services_package__srv__GetPointCloud2_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
common_services_package__srv__GetPointCloud2_Request__are_equal(const common_services_package__srv__GetPointCloud2_Request * lhs, const common_services_package__srv__GetPointCloud2_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
common_services_package__srv__GetPointCloud2_Request__copy(
  const common_services_package__srv__GetPointCloud2_Request * input,
  common_services_package__srv__GetPointCloud2_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

common_services_package__srv__GetPointCloud2_Request *
common_services_package__srv__GetPointCloud2_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  common_services_package__srv__GetPointCloud2_Request * msg = (common_services_package__srv__GetPointCloud2_Request *)allocator.allocate(sizeof(common_services_package__srv__GetPointCloud2_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(common_services_package__srv__GetPointCloud2_Request));
  bool success = common_services_package__srv__GetPointCloud2_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
common_services_package__srv__GetPointCloud2_Request__destroy(common_services_package__srv__GetPointCloud2_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    common_services_package__srv__GetPointCloud2_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
common_services_package__srv__GetPointCloud2_Request__Sequence__init(common_services_package__srv__GetPointCloud2_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  common_services_package__srv__GetPointCloud2_Request * data = NULL;

  if (size) {
    data = (common_services_package__srv__GetPointCloud2_Request *)allocator.zero_allocate(size, sizeof(common_services_package__srv__GetPointCloud2_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = common_services_package__srv__GetPointCloud2_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        common_services_package__srv__GetPointCloud2_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
common_services_package__srv__GetPointCloud2_Request__Sequence__fini(common_services_package__srv__GetPointCloud2_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      common_services_package__srv__GetPointCloud2_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

common_services_package__srv__GetPointCloud2_Request__Sequence *
common_services_package__srv__GetPointCloud2_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  common_services_package__srv__GetPointCloud2_Request__Sequence * array = (common_services_package__srv__GetPointCloud2_Request__Sequence *)allocator.allocate(sizeof(common_services_package__srv__GetPointCloud2_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = common_services_package__srv__GetPointCloud2_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
common_services_package__srv__GetPointCloud2_Request__Sequence__destroy(common_services_package__srv__GetPointCloud2_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    common_services_package__srv__GetPointCloud2_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
common_services_package__srv__GetPointCloud2_Request__Sequence__are_equal(const common_services_package__srv__GetPointCloud2_Request__Sequence * lhs, const common_services_package__srv__GetPointCloud2_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!common_services_package__srv__GetPointCloud2_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
common_services_package__srv__GetPointCloud2_Request__Sequence__copy(
  const common_services_package__srv__GetPointCloud2_Request__Sequence * input,
  common_services_package__srv__GetPointCloud2_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(common_services_package__srv__GetPointCloud2_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    common_services_package__srv__GetPointCloud2_Request * data =
      (common_services_package__srv__GetPointCloud2_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!common_services_package__srv__GetPointCloud2_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          common_services_package__srv__GetPointCloud2_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!common_services_package__srv__GetPointCloud2_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"
// Member `cloud`
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"

bool
common_services_package__srv__GetPointCloud2_Response__init(common_services_package__srv__GetPointCloud2_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    common_services_package__srv__GetPointCloud2_Response__fini(msg);
    return false;
  }
  // cloud
  if (!sensor_msgs__msg__PointCloud2__init(&msg->cloud)) {
    common_services_package__srv__GetPointCloud2_Response__fini(msg);
    return false;
  }
  return true;
}

void
common_services_package__srv__GetPointCloud2_Response__fini(common_services_package__srv__GetPointCloud2_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // cloud
  sensor_msgs__msg__PointCloud2__fini(&msg->cloud);
}

bool
common_services_package__srv__GetPointCloud2_Response__are_equal(const common_services_package__srv__GetPointCloud2_Response * lhs, const common_services_package__srv__GetPointCloud2_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // cloud
  if (!sensor_msgs__msg__PointCloud2__are_equal(
      &(lhs->cloud), &(rhs->cloud)))
  {
    return false;
  }
  return true;
}

bool
common_services_package__srv__GetPointCloud2_Response__copy(
  const common_services_package__srv__GetPointCloud2_Response * input,
  common_services_package__srv__GetPointCloud2_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // cloud
  if (!sensor_msgs__msg__PointCloud2__copy(
      &(input->cloud), &(output->cloud)))
  {
    return false;
  }
  return true;
}

common_services_package__srv__GetPointCloud2_Response *
common_services_package__srv__GetPointCloud2_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  common_services_package__srv__GetPointCloud2_Response * msg = (common_services_package__srv__GetPointCloud2_Response *)allocator.allocate(sizeof(common_services_package__srv__GetPointCloud2_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(common_services_package__srv__GetPointCloud2_Response));
  bool success = common_services_package__srv__GetPointCloud2_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
common_services_package__srv__GetPointCloud2_Response__destroy(common_services_package__srv__GetPointCloud2_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    common_services_package__srv__GetPointCloud2_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
common_services_package__srv__GetPointCloud2_Response__Sequence__init(common_services_package__srv__GetPointCloud2_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  common_services_package__srv__GetPointCloud2_Response * data = NULL;

  if (size) {
    data = (common_services_package__srv__GetPointCloud2_Response *)allocator.zero_allocate(size, sizeof(common_services_package__srv__GetPointCloud2_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = common_services_package__srv__GetPointCloud2_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        common_services_package__srv__GetPointCloud2_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
common_services_package__srv__GetPointCloud2_Response__Sequence__fini(common_services_package__srv__GetPointCloud2_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      common_services_package__srv__GetPointCloud2_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

common_services_package__srv__GetPointCloud2_Response__Sequence *
common_services_package__srv__GetPointCloud2_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  common_services_package__srv__GetPointCloud2_Response__Sequence * array = (common_services_package__srv__GetPointCloud2_Response__Sequence *)allocator.allocate(sizeof(common_services_package__srv__GetPointCloud2_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = common_services_package__srv__GetPointCloud2_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
common_services_package__srv__GetPointCloud2_Response__Sequence__destroy(common_services_package__srv__GetPointCloud2_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    common_services_package__srv__GetPointCloud2_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
common_services_package__srv__GetPointCloud2_Response__Sequence__are_equal(const common_services_package__srv__GetPointCloud2_Response__Sequence * lhs, const common_services_package__srv__GetPointCloud2_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!common_services_package__srv__GetPointCloud2_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
common_services_package__srv__GetPointCloud2_Response__Sequence__copy(
  const common_services_package__srv__GetPointCloud2_Response__Sequence * input,
  common_services_package__srv__GetPointCloud2_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(common_services_package__srv__GetPointCloud2_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    common_services_package__srv__GetPointCloud2_Response * data =
      (common_services_package__srv__GetPointCloud2_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!common_services_package__srv__GetPointCloud2_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          common_services_package__srv__GetPointCloud2_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!common_services_package__srv__GetPointCloud2_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "common_services_package/srv/detail/get_point_cloud2__functions.h"

bool
common_services_package__srv__GetPointCloud2_Event__init(common_services_package__srv__GetPointCloud2_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    common_services_package__srv__GetPointCloud2_Event__fini(msg);
    return false;
  }
  // request
  if (!common_services_package__srv__GetPointCloud2_Request__Sequence__init(&msg->request, 0)) {
    common_services_package__srv__GetPointCloud2_Event__fini(msg);
    return false;
  }
  // response
  if (!common_services_package__srv__GetPointCloud2_Response__Sequence__init(&msg->response, 0)) {
    common_services_package__srv__GetPointCloud2_Event__fini(msg);
    return false;
  }
  return true;
}

void
common_services_package__srv__GetPointCloud2_Event__fini(common_services_package__srv__GetPointCloud2_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  common_services_package__srv__GetPointCloud2_Request__Sequence__fini(&msg->request);
  // response
  common_services_package__srv__GetPointCloud2_Response__Sequence__fini(&msg->response);
}

bool
common_services_package__srv__GetPointCloud2_Event__are_equal(const common_services_package__srv__GetPointCloud2_Event * lhs, const common_services_package__srv__GetPointCloud2_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!common_services_package__srv__GetPointCloud2_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!common_services_package__srv__GetPointCloud2_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
common_services_package__srv__GetPointCloud2_Event__copy(
  const common_services_package__srv__GetPointCloud2_Event * input,
  common_services_package__srv__GetPointCloud2_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!common_services_package__srv__GetPointCloud2_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!common_services_package__srv__GetPointCloud2_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

common_services_package__srv__GetPointCloud2_Event *
common_services_package__srv__GetPointCloud2_Event__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  common_services_package__srv__GetPointCloud2_Event * msg = (common_services_package__srv__GetPointCloud2_Event *)allocator.allocate(sizeof(common_services_package__srv__GetPointCloud2_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(common_services_package__srv__GetPointCloud2_Event));
  bool success = common_services_package__srv__GetPointCloud2_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
common_services_package__srv__GetPointCloud2_Event__destroy(common_services_package__srv__GetPointCloud2_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    common_services_package__srv__GetPointCloud2_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
common_services_package__srv__GetPointCloud2_Event__Sequence__init(common_services_package__srv__GetPointCloud2_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  common_services_package__srv__GetPointCloud2_Event * data = NULL;

  if (size) {
    data = (common_services_package__srv__GetPointCloud2_Event *)allocator.zero_allocate(size, sizeof(common_services_package__srv__GetPointCloud2_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = common_services_package__srv__GetPointCloud2_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        common_services_package__srv__GetPointCloud2_Event__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
common_services_package__srv__GetPointCloud2_Event__Sequence__fini(common_services_package__srv__GetPointCloud2_Event__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      common_services_package__srv__GetPointCloud2_Event__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

common_services_package__srv__GetPointCloud2_Event__Sequence *
common_services_package__srv__GetPointCloud2_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  common_services_package__srv__GetPointCloud2_Event__Sequence * array = (common_services_package__srv__GetPointCloud2_Event__Sequence *)allocator.allocate(sizeof(common_services_package__srv__GetPointCloud2_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = common_services_package__srv__GetPointCloud2_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
common_services_package__srv__GetPointCloud2_Event__Sequence__destroy(common_services_package__srv__GetPointCloud2_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    common_services_package__srv__GetPointCloud2_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
common_services_package__srv__GetPointCloud2_Event__Sequence__are_equal(const common_services_package__srv__GetPointCloud2_Event__Sequence * lhs, const common_services_package__srv__GetPointCloud2_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!common_services_package__srv__GetPointCloud2_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
common_services_package__srv__GetPointCloud2_Event__Sequence__copy(
  const common_services_package__srv__GetPointCloud2_Event__Sequence * input,
  common_services_package__srv__GetPointCloud2_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(common_services_package__srv__GetPointCloud2_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    common_services_package__srv__GetPointCloud2_Event * data =
      (common_services_package__srv__GetPointCloud2_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!common_services_package__srv__GetPointCloud2_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          common_services_package__srv__GetPointCloud2_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!common_services_package__srv__GetPointCloud2_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
