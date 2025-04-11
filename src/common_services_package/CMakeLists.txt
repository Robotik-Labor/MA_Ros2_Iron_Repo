cmake_minimum_required(VERSION 3.8)
project(common_services_package)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Generate service files from srv/ directory
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetPlantpotCoords.srv"  # Add your .srv file here
  DEPENDENCIES geometry_msgs std_srvs  # Include dependencies that your service uses
)

ament_package()
