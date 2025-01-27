cmake_minimum_required(VERSION 3.8)
project(ppf_test_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL 1.4 REQUIRED)

#include_directories(/usr/local/include/pcl-1.14)
link_directories(/usr/local/lib)

message("PCL version: ${PCL_VERSION}")
message("PCL LIBRARIES: ${PCL_LIBRARIES}")
set(DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
)


add_executable(object_detection
  src/object_detection.cpp)

ament_target_dependencies(object_detection ${DEPENDENCIES})

target_link_libraries(object_detection ${PCL_LIBRARIES}
)

install(TARGETS
  object_detection
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
