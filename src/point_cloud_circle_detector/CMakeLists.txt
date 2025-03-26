cmake_minimum_required(VERSION 3.8)
project(point_cloud_circle_detector)


# Compiler warnings for GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set PCL paths
#set(PCL_DIR "/usr/local/share/pcl-1.14/cmake")  # Point to where PCLConfig.cmake is located
#set(PCL_ROOT "/usr/local/include/pcl-1.14")  # PCL header files

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)

# Specify the PCL version you want to use
#find_package(PCL 1.14.1 EXACT REQUIRED)



# Create executables
add_executable(point_cloud_circle_detector src/point_cloud_circle_detector.cpp)
add_executable(point_cloud_cylinder_detector src/point_cloud_cylinder_detector.cpp)
add_executable(point_cloud_plantpot_detector src/point_cloud_custom_plant_pot_detection.cpp)
add_executable(point_cloud_plantpot_ppf_detector src/point_cloud_plantpot_ppf_detector.cpp)
add_executable(point_cloud_plantpot_pfh_detector src/point_cloud_plantpot_pfh_detector.cpp)
add_executable(point_cloud_plantpot_edge_pfh_detector src/point_cloud_plantpot_edge_pfh_detector.cpp)
add_executable(point_cloud_plantpot_rgb_edge_pfh_detector src/point_cloud_plantpot_rgb_edge_pfh_detector.cpp)
add_executable(rgb_edge_filter src/rgb_edge_filter.cpp)
add_executable(cup_segmentation_color src/cup_segmentation_and_coloring.cpp)
add_executable(cup_pointcloud_segmentation_color src/cup_pointcloud_segmentation_and_coloring.cpp)



# Link libraries (add necessary PCL components)
ament_target_dependencies(point_cloud_circle_detector rclcpp sensor_msgs pcl_conversions)
ament_target_dependencies(point_cloud_cylinder_detector rclcpp sensor_msgs pcl_conversions)
ament_target_dependencies(point_cloud_plantpot_detector rclcpp sensor_msgs  pcl_conversions)
ament_target_dependencies(point_cloud_plantpot_ppf_detector rclcpp sensor_msgs pcl_conversions)
ament_target_dependencies(point_cloud_plantpot_pfh_detector rclcpp sensor_msgs pcl_conversions)
ament_target_dependencies(point_cloud_plantpot_edge_pfh_detector rclcpp sensor_msgs pcl_conversions)
ament_target_dependencies(point_cloud_plantpot_rgb_edge_pfh_detector rclcpp sensor_msgs pcl_conversions)
ament_target_dependencies(rgb_edge_filter rclcpp sensor_msgs pcl_conversions)
ament_target_dependencies(cup_segmentation_color rclcpp sensor_msgs pcl_conversions)
ament_target_dependencies(cup_pointcloud_segmentation_color rclcpp sensor_msgs pcl_conversions cv_bridge)

# Install targets
install(TARGETS point_cloud_circle_detector DESTINATION lib/${PROJECT_NAME})
install(TARGETS point_cloud_cylinder_detector DESTINATION lib/${PROJECT_NAME})
install(TARGETS point_cloud_plantpot_detector DESTINATION lib/${PROJECT_NAME})
install(TARGETS point_cloud_plantpot_ppf_detector DESTINATION lib/${PROJECT_NAME})
install(TARGETS point_cloud_plantpot_pfh_detector DESTINATION lib/${PROJECT_NAME})
install(TARGETS point_cloud_plantpot_edge_pfh_detector DESTINATION lib/${PROJECT_NAME})
install(TARGETS point_cloud_plantpot_rgb_edge_pfh_detector DESTINATION lib/${PROJECT_NAME})
install(TARGETS rgb_edge_filter DESTINATION lib/${PROJECT_NAME})
install(TARGETS cup_segmentation_color DESTINATION lib/${PROJECT_NAME})
install(TARGETS cup_pointcloud_segmentation_color DESTINATION lib/${PROJECT_NAME})


# Install pcd directory
install(
  DIRECTORY pcd
  DESTINATION share/${PROJECT_NAME}
)

target_link_libraries(point_cloud_plantpot_ppf_detector ${PCL_LIBRARIES}
)
target_link_libraries(point_cloud_plantpot_pfh_detector ${PCL_LIBRARIES}
)
target_link_libraries(point_cloud_plantpot_edge_pfh_detector ${PCL_LIBRARIES}
)
target_link_libraries(point_cloud_plantpot_rgb_edge_pfh_detector ${PCL_LIBRARIES}
)
target_link_libraries(rgb_edge_filter ${PCL_LIBRARIES}
)
target_link_libraries(cup_segmentation_color ${PCL_LIBRARIES}
)
target_link_libraries(cup_pointcloud_segmentation_color ${PCL_LIBRARIES}
)
# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # Skips copyright linting if not necessary
  set(ament_cmake_cpplint_FOUND TRUE)  # Skips cpplint if not in a git repo
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

