// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from common_services_package:srv/GetPlantpotCoords.idl
// generated code does not contain a copyright notice

#include "common_services_package/srv/detail/get_plantpot_coords__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_common_services_package
const rosidl_type_hash_t *
common_services_package__srv__GetPlantpotCoords__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x13, 0x1b, 0x0f, 0xad, 0xfe, 0x50, 0xb0, 0x63,
      0xd2, 0x1e, 0x8b, 0x81, 0xa8, 0x86, 0x69, 0x39,
      0x03, 0x76, 0x5a, 0x1d, 0x34, 0x94, 0x97, 0xbf,
      0x70, 0x0b, 0x2e, 0xea, 0xf2, 0x5a, 0xca, 0xe4,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_common_services_package
const rosidl_type_hash_t *
common_services_package__srv__GetPlantpotCoords_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa1, 0x17, 0xc6, 0x1c, 0xaa, 0x4b, 0x0c, 0x2e,
      0xd5, 0x65, 0xd4, 0xf9, 0x0b, 0xd4, 0xd2, 0x47,
      0x70, 0x6f, 0xfd, 0x74, 0xef, 0xb8, 0x91, 0x6d,
      0xc2, 0x9a, 0xb1, 0xa5, 0xb6, 0x7e, 0xbc, 0x1a,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_common_services_package
const rosidl_type_hash_t *
common_services_package__srv__GetPlantpotCoords_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe1, 0xc3, 0xed, 0xfc, 0x9f, 0x31, 0x06, 0xb7,
      0xfb, 0x3a, 0x52, 0xf4, 0x4b, 0x1c, 0x9e, 0xa4,
      0x91, 0x45, 0x2b, 0xe5, 0x79, 0xc3, 0x88, 0xf3,
      0x9c, 0x19, 0x88, 0x35, 0xd6, 0x74, 0x7c, 0xad,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_common_services_package
const rosidl_type_hash_t *
common_services_package__srv__GetPlantpotCoords_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x75, 0x82, 0x1b, 0xe4, 0x29, 0x35, 0xc9, 0xc2,
      0x2a, 0x13, 0x65, 0xd0, 0xfe, 0x05, 0x56, 0xff,
      0x28, 0x03, 0x5c, 0xc3, 0xbb, 0x43, 0x5e, 0x92,
      0xfd, 0xfd, 0xb3, 0xac, 0xe5, 0xbd, 0xb4, 0x0f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "geometry_msgs/msg/detail/point__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Point__EXPECTED_HASH = {1, {
    0x69, 0x63, 0x08, 0x48, 0x42, 0xa9, 0xb0, 0x44,
    0x94, 0xd6, 0xb2, 0x94, 0x1d, 0x11, 0x44, 0x47,
    0x08, 0xd8, 0x92, 0xda, 0x2f, 0x4b, 0x09, 0x84,
    0x3b, 0x9c, 0x43, 0xf4, 0x2a, 0x7f, 0x68, 0x81,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char common_services_package__srv__GetPlantpotCoords__TYPE_NAME[] = "common_services_package/srv/GetPlantpotCoords";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char common_services_package__srv__GetPlantpotCoords_Event__TYPE_NAME[] = "common_services_package/srv/GetPlantpotCoords_Event";
static char common_services_package__srv__GetPlantpotCoords_Request__TYPE_NAME[] = "common_services_package/srv/GetPlantpotCoords_Request";
static char common_services_package__srv__GetPlantpotCoords_Response__TYPE_NAME[] = "common_services_package/srv/GetPlantpotCoords_Response";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char common_services_package__srv__GetPlantpotCoords__FIELD_NAME__request_message[] = "request_message";
static char common_services_package__srv__GetPlantpotCoords__FIELD_NAME__response_message[] = "response_message";
static char common_services_package__srv__GetPlantpotCoords__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field common_services_package__srv__GetPlantpotCoords__FIELDS[] = {
  {
    {common_services_package__srv__GetPlantpotCoords__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {common_services_package__srv__GetPlantpotCoords_Request__TYPE_NAME, 53, 53},
    },
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPlantpotCoords__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {common_services_package__srv__GetPlantpotCoords_Response__TYPE_NAME, 54, 54},
    },
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPlantpotCoords__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {common_services_package__srv__GetPlantpotCoords_Event__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription common_services_package__srv__GetPlantpotCoords__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPlantpotCoords_Event__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPlantpotCoords_Request__TYPE_NAME, 53, 53},
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPlantpotCoords_Response__TYPE_NAME, 54, 54},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
common_services_package__srv__GetPlantpotCoords__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {common_services_package__srv__GetPlantpotCoords__TYPE_NAME, 45, 45},
      {common_services_package__srv__GetPlantpotCoords__FIELDS, 3, 3},
    },
    {common_services_package__srv__GetPlantpotCoords__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = common_services_package__srv__GetPlantpotCoords_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = common_services_package__srv__GetPlantpotCoords_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = common_services_package__srv__GetPlantpotCoords_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char common_services_package__srv__GetPlantpotCoords_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field common_services_package__srv__GetPlantpotCoords_Request__FIELDS[] = {
  {
    {common_services_package__srv__GetPlantpotCoords_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
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
common_services_package__srv__GetPlantpotCoords_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {common_services_package__srv__GetPlantpotCoords_Request__TYPE_NAME, 53, 53},
      {common_services_package__srv__GetPlantpotCoords_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char common_services_package__srv__GetPlantpotCoords_Response__FIELD_NAME__success[] = "success";
static char common_services_package__srv__GetPlantpotCoords_Response__FIELD_NAME__message[] = "message";
static char common_services_package__srv__GetPlantpotCoords_Response__FIELD_NAME__coordinates[] = "coordinates";

static rosidl_runtime_c__type_description__Field common_services_package__srv__GetPlantpotCoords_Response__FIELDS[] = {
  {
    {common_services_package__srv__GetPlantpotCoords_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPlantpotCoords_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPlantpotCoords_Response__FIELD_NAME__coordinates, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription common_services_package__srv__GetPlantpotCoords_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
common_services_package__srv__GetPlantpotCoords_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {common_services_package__srv__GetPlantpotCoords_Response__TYPE_NAME, 54, 54},
      {common_services_package__srv__GetPlantpotCoords_Response__FIELDS, 3, 3},
    },
    {common_services_package__srv__GetPlantpotCoords_Response__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char common_services_package__srv__GetPlantpotCoords_Event__FIELD_NAME__info[] = "info";
static char common_services_package__srv__GetPlantpotCoords_Event__FIELD_NAME__request[] = "request";
static char common_services_package__srv__GetPlantpotCoords_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field common_services_package__srv__GetPlantpotCoords_Event__FIELDS[] = {
  {
    {common_services_package__srv__GetPlantpotCoords_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPlantpotCoords_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {common_services_package__srv__GetPlantpotCoords_Request__TYPE_NAME, 53, 53},
    },
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPlantpotCoords_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {common_services_package__srv__GetPlantpotCoords_Response__TYPE_NAME, 54, 54},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription common_services_package__srv__GetPlantpotCoords_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPlantpotCoords_Request__TYPE_NAME, 53, 53},
    {NULL, 0, 0},
  },
  {
    {common_services_package__srv__GetPlantpotCoords_Response__TYPE_NAME, 54, 54},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
common_services_package__srv__GetPlantpotCoords_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {common_services_package__srv__GetPlantpotCoords_Event__TYPE_NAME, 51, 51},
      {common_services_package__srv__GetPlantpotCoords_Event__FIELDS, 3, 3},
    },
    {common_services_package__srv__GetPlantpotCoords_Event__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = common_services_package__srv__GetPlantpotCoords_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = common_services_package__srv__GetPlantpotCoords_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
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
  "geometry_msgs/Point coordinates";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
common_services_package__srv__GetPlantpotCoords__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {common_services_package__srv__GetPlantpotCoords__TYPE_NAME, 45, 45},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 226, 226},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
common_services_package__srv__GetPlantpotCoords_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {common_services_package__srv__GetPlantpotCoords_Request__TYPE_NAME, 53, 53},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
common_services_package__srv__GetPlantpotCoords_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {common_services_package__srv__GetPlantpotCoords_Response__TYPE_NAME, 54, 54},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
common_services_package__srv__GetPlantpotCoords_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {common_services_package__srv__GetPlantpotCoords_Event__TYPE_NAME, 51, 51},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
common_services_package__srv__GetPlantpotCoords__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *common_services_package__srv__GetPlantpotCoords__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *common_services_package__srv__GetPlantpotCoords_Event__get_individual_type_description_source(NULL);
    sources[3] = *common_services_package__srv__GetPlantpotCoords_Request__get_individual_type_description_source(NULL);
    sources[4] = *common_services_package__srv__GetPlantpotCoords_Response__get_individual_type_description_source(NULL);
    sources[5] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
common_services_package__srv__GetPlantpotCoords_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *common_services_package__srv__GetPlantpotCoords_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
common_services_package__srv__GetPlantpotCoords_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *common_services_package__srv__GetPlantpotCoords_Response__get_individual_type_description_source(NULL),
    sources[1] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
common_services_package__srv__GetPlantpotCoords_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *common_services_package__srv__GetPlantpotCoords_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *common_services_package__srv__GetPlantpotCoords_Request__get_individual_type_description_source(NULL);
    sources[3] = *common_services_package__srv__GetPlantpotCoords_Response__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
