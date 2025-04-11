#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>   // For trigonometrical functions
#include <tf2/LinearMath/Quaternion.h>
#include <common_services_package/srv/get_plantpot_coords.hpp>   // Import service message - adjust to your actual service type

#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, sensor_msgs::msg::Image::SharedPtr> 
filterGreenPointCloud(
    const sensor_msgs::msg::Image::SharedPtr rgb_msg,
    const sensor_msgs::msg::Image::SharedPtr  depth_msg,
    const sensor_msgs::msg::CameraInfo& camera_info) {
    
    int h_min = 30;
    int h_max = 90;
    int s_min = 65;
    int s_max = 210;
    int v_min = 40;
    int v_max = 255;


    // Convert ROS Image message to OpenCV Mat
    cv::Mat rgb_image;
    try {
        rgb_image = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("sensor_probe_node"), "Could not convert RGB image: %s", e.what());
        return {nullptr, nullptr};
    }
    cv::Mat depth_image;
    try {
        depth_image = cv_bridge::toCvCopy(depth_msg, "16UC1")->image;
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("sensor_probe_node"), "Could not convert depth image: %s", e.what());
        return {nullptr, nullptr};
    }

    
    // Apply green color filtering using OpenCV
    cv::Mat hsv_image, green_mask;
    cv::cvtColor(rgb_image, hsv_image, cv::COLOR_BGR2HSV);
    
    // Define the HSV range for green color detection
    cv::Scalar lower_green(h_min, s_min, v_min); // Lower bound for green
    cv::Scalar upper_green(h_max, s_max, v_max); // Upper bound for green
    
    // Threshold the HSV image to get only the green areas
    cv::inRange(hsv_image, lower_green, upper_green, green_mask);
    
    // Apply the mask to the original RGB image
    cv::Mat filtered_rgb;
    cv::bitwise_and(rgb_image, rgb_image, filtered_rgb, green_mask);
    
    // Convert back to RGB (OpenCV loads as BGR by default)
    cv::Mat final_rgb;
    cv::cvtColor(filtered_rgb, final_rgb, cv::COLOR_BGR2RGB);
    
    // Create output image message
    sensor_msgs::msg::Image::SharedPtr output_img = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", final_rgb).toImageMsg();
    
    // Create a point cloud based on depth and green-filtered RGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud->width = depth_image.cols;
    cloud->height = depth_image.rows;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    
    for (int v = 0; v < depth_image.rows; ++v) {
        for (int u = 0; u < depth_image.cols; ++u) {
            uint16_t depth_value = depth_image.at<uint16_t>(v, u);
            
            // Skip invalid depth values (0 means no depth data)
            if (depth_value == 0) continue;
            
            // Convert depth to meters
            float z = depth_value * 0.001f;

            float x = (depth_image.width - u - camera_info.k[2]) * z / camera_info.k[0];
            float y = (depth_image.height - v - camera_info.k[5]) * z / camera_info.k[4];
            
            // Get the RGB value from the original image
            cv::Vec3b rgb = rgb_image.at<cv::Vec3b>(v, u);
            
            // Check if the pixel is within the green mask
            if (green_mask.at<uchar>(v, u) != 0) {
                pcl::PointXYZRGB point;
                point.x = x;
                point.y = y;
                point.z = z;
                
                // Set the color of the point based on the filtered green area
                point.r = rgb[2]; // Red
                point.g = rgb[1]; // Green
                point.b = rgb[0]; // Blue
                
                // Add the point to the point cloud
                cloud->points[v * depth_image.cols + u] = point;
            }
        }
    }
    
    return {cloud, output_img};
}


int main(int argc, char **argv) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
      "sensor_probe_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("sensor_probe_node");
  
  // Create the MoveGroup interface for the UR5 arm
  using moveit::planning_interface::MoveGroupInterface;
  moveit::planning_interface::MoveGroupInterface::Options options(
      "ur5_arm", "robot_description", "/UR5");
  auto move_group_interface = MoveGroupInterface(node, options);
  move_group_interface.setPlanningTime(20.0);
  move_group_interface.setMaxVelocityScalingFactor(0.1);
  move_group_interface.setMaxAccelerationScalingFactor(0.1);
  move_group_interface.setPlannerId("BiTRRT");
  move_group_interface.setNumPlanningAttempts(20);
  move_group_interface.setGoalJointTolerance(0.03);

  // Create a publisher for visualization markers
  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
    "UR5/probe_points", rclcpp::QoS(10));

  // PointCloud publisher
  auto pointcloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/UR5/UR5_camera/depth/color/points/green_filtered", 1);
        
  // Image publisher
  auto image_pub = create_publisher<sensor_msgs::msg::Image>("/UR5/image/green_filtered", 1);

  auto depth_info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/UR5/UR5_camera/aligned_depth_to_color/camera_info",
    rclcpp::QoS(1).best_effort(),
    [](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        // Process camera info directly
    });

  auto depth_sub = node->create_subscription<sensor_msgs::msg::Image>(
    "/UR5/UR5_camera/aligned_depth_to_color/image_raw",
    rclcpp::QoS(1).best_effort(),
    [](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        // Process depth image directly
    });

  auto rgb_sub = node->create_subscription<sensor_msgs::msg::Image>(
    "/UR5/UR5_camera/color/image_raw",
    rclcpp::QoS(1).best_effort(),
    [](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        // Process RGB image directly (e.g., convert to OpenCV image and filter)
    });



  // Vector to store target poses
  std::vector<geometry_msgs::msg::Pose> target_poses;
  
  // Create a service client to get the last point
  auto client = node->create_client<common_services_package::srv::GetPlantpotCoords>("/UR5/service/plantpot_coords");

  // Convertig an pointcloud to world coordinates and getting the highest coordindate 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  auto filtered_image
  filtered_pointcloud , filtered_image = filterGreenPointCloud( rgb_sub, depth_sub,depth_info_sub);

  Eigen::Matrix4f camera_to_world
  // Look up transform from camera frame to world frame
  geometry_msgs::msg::TransformStamped transform_stamped = 
      tf_buffer_->lookupTransform("world", "UR5_camera_depth_optical_frame", rclcpp::Time(0), std::chrono::seconds(1));
        
  // Convert to Eigen::Matrix4f
  Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform_stamped.transform);
  Eigen::Matrix4f camera_to_world = transform_eigen.matrix().cast<float>();
        
  RCLCPP_INFO(this->get_logger(), "Got transform from camera to world");
	// Transform filtered cloud from camera frame to world frame
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*filtered_pointcloud, *cloud_world, camera_to_world);

  float max_height = -999999999.0;  // Initialize with a large value
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>


		    // Check if this radius is smaller than the current smallest radius
		    if (z > max_height) {
			   max_height = z; 
		    }
		}



  // Wait for the service to be available
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger, "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(logger, "Waiting for service to appear...");
  }

  // Create a request
  auto request = std::make_shared<common_services_package::srv::GetPlantpotCoords::Request>();
  
  // Send the request
  auto result_future = client->async_send_request(request);
  
  // Wait for the result
  if (rclcpp::spin_until_future_complete(node, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto msg = result_future.get()->coordinates;
    RCLCPP_INFO(logger, "Received point: x=%.3f, y=%.3f, z=%.3f", msg.x, msg.y, msg.z);
    
    // Center of the semicircle
    double center_x = msg.x;
    double center_y = msg.y;
    double center_z = z + 0.1;

    // Define the radius of the semicircle
    const double radius = 0.24;
    const int num_positions = 5;
    const double angle_step = M_PI / (num_positions - 1);  // Divide the semicircle into equal parts

    //calculation Point nearest (0,0,z)
    double start_angle = atan2(center_y, center_x) + M_PI;

    double Angles[5] = {start_angle - M_PI/2, start_angle - M_PI/4, start_angle, start_angle + M_PI/4, start_angle + M_PI/2};
    
    const float colors[10][3] = {
      {1.0f, 0.0f, 0.0f},     // Red
      {0.0f, 1.0f, 0.0f},     // Green
      {0.0f, 0.0f, 1.0f},     // Blue
      {1.0f, 1.0f, 0.0f},     // Yellow
      {1.0f, 0.0f, 1.0f},     // Magenta
      {0.0f, 1.0f, 1.0f},     // Cyan
      {0.5f, 0.0f, 0.0f},     // Dark Red
      {0.0f, 0.5f, 0.0f},     // Dark Green
      {0.0f, 0.0f, 0.5f},     // Dark Blue
      {1.0f, 0.5f, 0.0f}      // Orange
    };

    // Calculate the 5 positions
    for (int i = 0; i < num_positions; ++i) {
      double x = center_x + radius * cos(Angles[i]);
      double y = center_y + radius * sin(Angles[i]);

      // Define target pose (Y-axis aligned with the world negative Z-axis)
      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = x;
      target_pose.position.y = y;
      target_pose.position.z = center_z;

      double vx = center_x - x;
      double vy = center_y - y;
      double vz = 0.5 - z;

      double Norm = sqrt(vx*vx + vy*vy + vz*vz);

      // Step 3: Define the reference vector (assuming robot arm points along Z-axis)
      double rx = 0, ry = 0, rz = 1; // World Z-axis

      // Step 4: Compute the cross product (axis of rotation)
      double axis_x = ry * vz/Norm  - rz * vy/Norm;
      double axis_y = rz * vx/Norm - rx * vz/Norm;
      double axis_z = rx * vy/Norm - ry * vx/Norm;

      double dotProduct = rx * vx + ry * vy + rz * vz;
      double angle = acos(dotProduct); // Angle between the two vectors

      // Normalize the rotation axis
      double norm = sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z);
      axis_x /= norm;
      axis_y /= norm;
      axis_z /= norm;

      target_pose.orientation.x = axis_x * sin(angle / 2);
      target_pose.orientation.y = axis_y * sin(angle / 2);  // Y-axis aligned with world -Z-axis
      target_pose.orientation.z = axis_z * sin(angle / 2);
      target_pose.orientation.w = cos(angle / 2);  // Identity quaternion

      tf2::Quaternion arm_orientation;
      arm_orientation.setX(target_pose.orientation.x);
      arm_orientation.setY(target_pose.orientation.y);
      arm_orientation.setZ(target_pose.orientation.z);
      arm_orientation.setW(target_pose.orientation.w);

      double angle_to_rotate_z = atan2(vy, vx);

      // Define the fixed camera rotation quaternion
      tf2::Quaternion camera_rotation;
      camera_rotation.setRPY(0.0, 0.0, angle_to_rotate_z);

      // Combine the two quaternions
      tf2::Quaternion final_orientation = arm_orientation * camera_rotation;

      // Set the final orientation to the target pose
      target_pose.orientation.x = final_orientation.x();
      target_pose.orientation.y = final_orientation.y();
      target_pose.orientation.z = final_orientation.z();
      target_pose.orientation.w = final_orientation.w();

      float r = colors[i % 10][0];
      float g = colors[i % 10][1];
      float b = colors[i % 10][2];

      // Create a marker to visualize the target position in RViz
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = rclcpp::Clock().now();
      marker.ns = "target_positions";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = target_pose.position;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = 1.0;

      // Publish the marker
      marker_pub->publish(marker);
      
      geometry_msgs::msg::Pose above_target_pose;
      above_target_pose = target_pose;
      above_target_pose.position.z = center_z + 0.07;
      
      // Create a marker to visualize the target position in RViz
      visualization_msgs::msg::Marker marker_top;
      marker_top.header.frame_id = "world";
      marker_top.header.stamp = rclcpp::Clock().now();
      marker_top.ns = "target_positions";
      marker_top.id = i + num_positions;
      marker_top.type = visualization_msgs::msg::Marker::SPHERE;
      marker_top.action = visualization_msgs::msg::Marker::ADD;
      marker_top.pose.position = above_target_pose.position;
      marker_top.pose.orientation.w = 1.0;
      marker_top.scale.x = 0.05;
      marker_top.scale.y = 0.05;
      marker_top.scale.z = 0.05;
      marker_top.color.r = r;
      marker_top.color.g = g;
      marker_top.color.b = b;
      marker_top.color.a = 1.0;
      
      // Publish the marker
      marker_pub->publish(marker_top);
      
      target_poses.push_back(above_target_pose);
      target_poses.push_back(target_pose);
    }

    RCLCPP_INFO(logger, "Positions calculated, moving to target positions...");
  } else {
    RCLCPP_ERROR(logger, "Failed to call service");
    return 1;
  }
  
  
  moveit_msgs::msg::JointConstraint shoulder_pan_constraint;
  shoulder_pan_constraint.joint_name = "UR5_shoulder_pan_joint"; // Name of the joint
  shoulder_pan_constraint.position = M_PI / 2; // Center position (0 radians)
  shoulder_pan_constraint.tolerance_above = M_PI/2 ; // ±180 degrees tolerance
  shoulder_pan_constraint.tolerance_below = M_PI/2 ; // ±180 degrees tolerance
  shoulder_pan_constraint.weight = 0.8; // Explicitly setting weight to 1.0 for importance

  // Create joint constraints for the wrist joints
  moveit_msgs::msg::JointConstraint wrist_1_constraint;
  wrist_1_constraint.joint_name = "UR5_wrist_1_joint"; // Name of the joint
  wrist_1_constraint.position = 0.0; // Center position (0 radians)
  wrist_1_constraint.tolerance_above = M_PI; // ±180 degrees tolerance
  wrist_1_constraint.tolerance_below = M_PI; // ±180 degrees tolerance
  wrist_1_constraint.weight = 0.8; // Explicitly setting weight to 1.0 for importance

  moveit_msgs::msg::JointConstraint wrist_2_constraint;
  wrist_2_constraint.joint_name = "UR5_wrist_2_joint"; // Name of the joint
  wrist_2_constraint.position = M_PI/2;         // Center position (0 radians)
  wrist_2_constraint.tolerance_above = M_PI; // ±180 degrees tolerance
  wrist_2_constraint.tolerance_below = M_PI; // ±180 degrees tolerance
  wrist_2_constraint.weight = 0.8; // Explicitly setting weight to 1.0 for importance

  moveit_msgs::msg::JointConstraint wrist_3_constraint;
  wrist_3_constraint.joint_name = "UR5_wrist_3_joint"; // Name of the joint
  wrist_3_constraint.position = M_PI;         // Center position (0 radians)
  wrist_3_constraint.tolerance_above = M_PI/2; // ±180 degrees tolerance
  wrist_3_constraint.tolerance_below = M_PI/2; // ±180 degrees tolerance
  wrist_3_constraint.weight = 0.8; // Explicitly setting weight to 1.0 for importance

  moveit_msgs::msg::Constraints constraints;
  //constraints.joint_constraints.push_back(shoulder_pan_constraint);
  constraints.joint_constraints.push_back(wrist_1_constraint);
  constraints.joint_constraints.push_back(wrist_2_constraint);
  constraints.joint_constraints.push_back(wrist_3_constraint);
  move_group_interface.setPathConstraints(constraints);

  
  bool success;
  moveit::planning_interface::MoveGroupInterface::Plan plan_to_above;
  moveit::planning_interface::MoveGroupInterface::Plan plan_linar_down;
  moveit_msgs::msg::RobotTrajectory trajectory_down;
  geometry_msgs::msg::Pose probe_point;
  const double eef_step = 0.01;
  double fraction;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose start_pose;
  std::vector<double> target_joint_degrees = {90.0, -156.0, 103.0, 80.0, 91.0, 0.0};  // Example joint values in degrees
  std::vector<double> radians;
  for (double degree : target_joint_degrees) {
        radians.push_back(degree * M_PI / 180.0);  // Convert to radians
    }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  
  for (int i = 0; i < target_poses.size(); i++) {
      RCLCPP_INFO(logger, "Motion_planning_start");
      move_group_interface.setPoseTarget(target_poses[i]);

      success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (!success){ continue;}
      move_group_interface.execute(plan);

      filtered_pointcloud , filtered_image = filterGreenPointCloud( rgb_sub, depth_sub,depth_info_sub);

      pointcloud_pub->publish(filtered_pointcloud)
  }
  
  move_group_interface.setNamedTarget("Home");
  success = (move_group_interface.plan(plan_to_home) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group_interface.execute(plan_to_home);
  
  // Shutdown ROS after the task is finished
  rclcpp::shutdown();
  return 1;
}
