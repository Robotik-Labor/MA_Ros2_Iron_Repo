// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from common_services_package:srv/GetPointCloud2.idl
// generated code does not contain a copyright notice

#include "common_services_package/srv/detail/get_point_cloud2__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_common_services_package
const rosidl_type_hash_t *
common_services_package__srv__GetPointCloud2__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7c, 0xc9, 0xe8, 0x9e, 0xda, 0x7e, 0xa8, 0x97,
      0x62, 0x85, 0x57, 0xd3, 0x8b, 0x41, 0x94, 0xd6,
      0x91, 0x70, 0xe5, 0x53, 0x1a, 0x0b, 0x1e, 0xb3,
      0x87, 0x8f, 0x30, 0x23, 0x33, 0xd1, 0xa6, 0xb3,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_common_services_package
const rosidl_type_hash_t *
common_services_package__srv__GetPointCloud2_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x99, 0x55, 0x40, 0x2f, 0xb5, 0xa6, 0xb0, 0x87,
      0x28, 0x1f, 0xd6, 0x49, 0x16, 0x2d, 0x72, 0x47,
      0x8d, 0xe2, 0xd8, 0xed, 0x63, 0xe6, 0x41, 0xd6,
      0x33, 0x86, 0x26, 0x1d, 0x7b, 0xf6, 0x43, 0x64,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_common_services_package
const rosidl_type_hash_t *
common_services_package__srv__GetPointCloud2_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1f, 0x3d, 0x01, 0x38, 0xd2, 0x70, 0x71, 0x3c,
      0x82, 0xcf, 0x77, 0x59, 0x0a, 0xca, 0x59, 0x6d,
      0xa2, 0x5f, 0x16, 0x4f, 0x1a, 0x1a, 0x3d, 0xcc,
      0xe3, 0xa7, 0x0f, 0x98, 0xbe, 0x50, 0x9a, 0x33,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_common_services_package
const rosidl_type_hash_t *
common_services_package__srv__GetPointCloud2_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0d, 0xef, 0xae, 0xcc, 0x85, 0xa1, 0x8f, 0xd5,
      0x93, 0xdc, 0x8f, 0x18, 0xaf, 0x74, 0x52, 0x12,
      0xd6, 0x5b, 0xf6, 0x8f, 0x07, 0x16, 0xdd, 0xda,
      0xbd, 0x12, 0xbd, 0x74, 0x43, 0xb8, 0xcc, 0x6e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "sensor_msgs/msg/detail/point_field__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t sensor_msgs__msg__PointCloud2__EXPECTED_HASH = {1, {
    0x91, 0x98, 0xca, 0xbf, 0x7d, 0xa3, 0x79, 0x6a,
    0xe6, 0xfe, 0x19, 0xc4, 0xcb, 0x3b, 0xdd, 0x35,
    0x25, 0x49, 0x29, 0x88, 0xc7, 0x05, 0x22, 0x62,
    0x8a, 0xf5, 0xda, 0xa1, 0x24, 0xba, 0xe2, 0xb5,
  }};
static const rosidl_type_hash_t sensor_msgs__msg__PointField__EXPECTED_HASH = {1, {
    0x5c, 0x6a, 0x47, 0x50, 0x72, 0x8c, 0x2b, 0xcf,
    0xbb, 0xf7, 0x03, 0x72, 0x25, 0xb2, 0x0b, 0x02,
    0xd4, 0x42, 0x96, 0x34, 0x73, 0x21, 0x46, 0xb7,
    0x42, 0xde, 0xe1, 0x72, 0x66, 0x37, 0xef, 0x01,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char common_services_package__srv__GetPointCloud2__TYPE_NAME[] = "common_services_package/srv/GetPointCloud2";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char common_services_package__srv__GetPointCloud2_Event__TYPE_NAME[] = "common_services_package/srv/GetPointCloud2_Event";
static char common_services_package__srv__GetPointCloud2_Request__TYPE_NAME[] = "common_services_package/srv/GetPointCloud2_Request";
static char common_services_package__srv__GetPointCloud2_Response__TYPE_NAME[] = "common_services_package/srv/GetPointCloud2_Response";
static char sensor_msgs__msg__PointCloud2__TYPE_NAME[] = "sensor_msgs/msg/PointCloud2";
static char sensor_msgs__msg__PointField__TYPE_NAME[] = "sensor_msgs/msg/PointField";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char common_services_package__srv__GetPointCloud2__FIELD_NAME__request_message[] = "request_message";
static char common_services_package__srv__GetPointCloud2__FIELD_NAME__response_message[] = "response_message";
static char common_services_package__srv__GetPointCloud2__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field common_services_package__srv__GetPointCloud2__FIELDS[] = {
  {
    {common_services_package__srv__GetPointCloud2__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {common_services_package__srv__GetPointCloud2_Request__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPointCloud2__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {common_services_package__srv__GetPointCloud2_Response__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPointCloud2__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {common_services_package__srv__GetPointCloud2_Event__TYPE_NAME, 48, 48},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription common_services_package__srv__GetPointCloud2__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPointCloud2_Event__TYPE_NAME, 48, 48},
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPointCloud2_Request__TYPE_NAME, 50, 50},
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPointCloud2_Response__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {sensor_msgs__msg__PointCloud2__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {sensor_msgs__msg__PointField__TYPE_NAME, 26, 26},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
common_services_package__srv__GetPointCloud2__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {common_services_package__srv__GetPointCloud2__TYPE_NAME, 42, 42},
      {common_services_package__srv__GetPointCloud2__FIELDS, 3, 3},
    },
    {common_services_package__srv__GetPointCloud2__REFERENCED_TYPE_DESCRIPTIONS, 8, 8},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = common_services_package__srv__GetPointCloud2_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = common_services_package__srv__GetPointCloud2_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = common_services_package__srv__GetPointCloud2_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&sensor_msgs__msg__PointCloud2__EXPECTED_HASH, sensor_msgs__msg__PointCloud2__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = sensor_msgs__msg__PointCloud2__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&sensor_msgs__msg__PointField__EXPECTED_HASH, sensor_msgs__msg__PointField__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = sensor_msgs__msg__PointField__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char common_services_package__srv__GetPointCloud2_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field common_services_package__srv__GetPointCloud2_Request__FIELDS[] = {
  {
    {common_services_package__srv__GetPointCloud2_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
common_services_package__srv__GetPointCloud2_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {common_services_package__srv__GetPointCloud2_Request__TYPE_NAME, 50, 50},
      {common_services_package__srv__GetPointCloud2_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char common_services_package__srv__GetPointCloud2_Response__FIELD_NAME__success[] = "success";
static char common_services_package__srv__GetPointCloud2_Response__FIELD_NAME__message[] = "message";
static char common_services_package__srv__GetPointCloud2_Response__FIELD_NAME__cloud[] = "cloud";

static rosidl_runtime_c__type_description__Field common_services_package__srv__GetPointCloud2_Response__FIELDS[] = {
  {
    {common_services_package__srv__GetPointCloud2_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPointCloud2_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPointCloud2_Response__FIELD_NAME__cloud, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {sensor_msgs__msg__PointCloud2__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription common_services_package__srv__GetPointCloud2_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {sensor_msgs__msg__PointCloud2__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {sensor_msgs__msg__PointField__TYPE_NAME, 26, 26},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
common_services_package__srv__GetPointCloud2_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {common_services_package__srv__GetPointCloud2_Response__TYPE_NAME, 51, 51},
      {common_services_package__srv__GetPointCloud2_Response__FIELDS, 3, 3},
    },
    {common_services_package__srv__GetPointCloud2_Response__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&sensor_msgs__msg__PointCloud2__EXPECTED_HASH, sensor_msgs__msg__PointCloud2__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = sensor_msgs__msg__PointCloud2__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&sensor_msgs__msg__PointField__EXPECTED_HASH, sensor_msgs__msg__PointField__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = sensor_msgs__msg__PointField__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char common_services_package__srv__GetPointCloud2_Event__FIELD_NAME__info[] = "info";
static char common_services_package__srv__GetPointCloud2_Event__FIELD_NAME__request[] = "request";
static char common_services_package__srv__GetPointCloud2_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field common_services_package__srv__GetPointCloud2_Event__FIELDS[] = {
  {
    {common_services_package__srv__GetPointCloud2_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPointCloud2_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {common_services_package__srv__GetPointCloud2_Request__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPointCloud2_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {common_services_package__srv__GetPointCloud2_Response__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription common_services_package__srv__GetPointCloud2_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPointCloud2_Request__TYPE_NAME, 50, 50},
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPointCloud2_Response__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {sensor_msgs__msg__PointCloud2__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {sensor_msgs__msg__PointField__TYPE_NAME, 26, 26},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
common_services_package__srv__GetPointCloud2_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {common_services_package__srv__GetPointCloud2_Event__TYPE_NAME, 48, 48},
      {common_services_package__srv__GetPointCloud2_Event__FIELDS, 3, 3},
    },
    {common_services_package__srv__GetPointCloud2_Event__REFERENCED_TYPE_DESCRIPTIONS, 7, 7},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = common_services_package__srv__GetPointCloud2_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = common_services_package__srv__GetPointCloud2_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&sensor_msgs__msg__PointCloud2__EXPECTED_HASH, sensor_msgs__msg__PointCloud2__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = sensor_msgs__msg__PointCloud2__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&sensor_msgs__msg__PointField__EXPECTED_HASH, sensor_msgs__msg__PointField__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = sensor_msgs__msg__PointField__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Request: No input, the service will just return the last known coordinates\n"
  "---\n"
  "# Response: success (bool), message (string), and the last known coordinates (Point)\n"
  "bool success\n"
  "string message\n"
  "sensor_msgs/PointCloud2 cloud\n"
  "";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
common_services_package__srv__GetPointCloud2__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {common_services_package__srv__GetPointCloud2__TYPE_NAME, 42, 42},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 225, 225},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
common_services_package__srv__GetPointCloud2_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {common_services_package__srv__GetPointCloud2_Request__TYPE_NAME, 50, 50},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
common_services_package__srv__GetPointCloud2_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {common_services_package__srv__GetPointCloud2_Response__TYPE_NAME, 51, 51},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
common_services_package__srv__GetPointCloud2_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {common_services_package__srv__GetPointCloud2_Event__TYPE_NAME, 48, 48},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
common_services_package__srv__GetPointCloud2__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[9];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 9, 9};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *common_services_package__srv__GetPointCloud2__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *common_services_package__srv__GetPointCloud2_Event__get_individual_type_description_source(NULL);
    sources[3] = *common_services_package__srv__GetPointCloud2_Request__get_individual_type_description_source(NULL);
    sources[4] = *common_services_package__srv__GetPointCloud2_Response__get_individual_type_description_source(NULL);
    sources[5] = *sensor_msgs__msg__PointCloud2__get_individual_type_description_source(NULL);
    sources[6] = *sensor_msgs__msg__PointField__get_individual_type_description_source(NULL);
    sources[7] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[8] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
common_services_package__srv__GetPointCloud2_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *common_services_package__srv__GetPointCloud2_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
common_services_package__srv__GetPointCloud2_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *common_services_package__srv__GetPointCloud2_Response__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *sensor_msgs__msg__PointCloud2__get_individual_type_description_source(NULL);
    sources[3] = *sensor_msgs__msg__PointField__get_individual_type_description_source(NULL);
    sources[4] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
common_services_package__srv__GetPointCloud2_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[8];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 8, 8};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *common_services_package__srv__GetPointCloud2_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *common_services_package__srv__GetPointCloud2_Request__get_individual_type_description_source(NULL);
    sources[3] = *common_services_package__srv__GetPointCloud2_Response__get_individual_type_description_source(NULL);
    sources[4] = *sensor_msgs__msg__PointCloud2__get_individual_type_description_source(NULL);
    sources[5] = *sensor_msgs__msg__PointField__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[7] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
