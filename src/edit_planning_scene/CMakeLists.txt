cmake_minimum_required(VERSION 3.8)
project(edit_planning_scene)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(add_object_to_planning_scene_node src/add_object_to_planning_scene.cpp)
add_executable(pointcloud_to_octomap src/octo_map.cpp)

ament_target_dependencies(add_object_to_planning_scene_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  visualization_msgs
)

ament_target_dependencies(pointcloud_to_octomap
  rclcpp
  sensor_msgs
  octomap
  octomap_msgs
  pcl_conversions
  tf2_ros
  tf2_eigen
  tf2_sensor_msgs
  moveit_msgs
  moveit_ros_planning_interface
)

target_link_libraries(pointcloud_to_octomap
  ${PCL_LIBRARIES}
)

install(TARGETS pointcloud_to_octomap
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  add_object_to_planning_scene_node
  DESTINATION lib/${PROJECT_NAME}
)

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
