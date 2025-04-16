#define EIGEN_DONT_ALIGN_STATICALLY
#define EIGEN_DONT_VECTORIZE

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>   // For trigonometrical functions
#include <tf2/LinearMath/Quaternion.h>
#include <common_services_package/srv/get_plantpot_coords.hpp>   // Import service message

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>


std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, sensor_msgs::msg::Image::SharedPtr> 
filterGreenPointCloud(
    const sensor_msgs::msg::Image::SharedPtr rgb_msg,
    const sensor_msgs::msg::Image::SharedPtr depth_msg,
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
    
    int h_min = 30;
    int h_max = 75;
    int s_min = 65;
    int s_max = 210;
    int v_min = 40;
    int v_max = 255;

    // Convert ROS Image message to OpenCV Mat
    cv::Mat rgb_image;
    try {
        rgb_image = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("create_octomap_node"), "Could not convert RGB image: %s", e.what());
        return {nullptr, nullptr};
    }
    cv::Mat depth_image;
    try {
        depth_image = cv_bridge::toCvCopy(depth_msg, "16UC1")->image;   
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("create_octomap_node"), "Could not convert depth image: %s", e.what());
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
            if (depth_value <= 20) continue;
            
            // Convert depth to meters
            float z = depth_value * 0.001f;

            float x = ( u - camera_info->k[2]) * z / camera_info->k[0];
            float y = ( v - camera_info->k[5]) * z / camera_info->k[4];
            
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

class SensorProbeNode : public rclcpp::Node {
public:
    SensorProbeNode() : Node("create_octomap_node") {
        // Set up TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create publishers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("UR5/probe_points", 10);
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/UR5/UR5_camera/depth/color/points/green_filtered", rclcpp::QoS(10).transient_local());
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/UR5/image/green_filtered", 1);
        
        // Create subscribers
        depth_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/UR5/UR5_camera/aligned_depth_to_color/camera_info",
            rclcpp::SensorDataQoS(),
            std::bind(&SensorProbeNode::cameraInfoCallback, this, std::placeholders::_1));
            
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/UR5/UR5_camera/aligned_depth_to_color/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&SensorProbeNode::depthImageCallback, this, std::placeholders::_1));
            
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/UR5/UR5_camera/color/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&SensorProbeNode::rgbImageCallback, this, std::placeholders::_1));
        
        cloud_message_publisher_ = this->create_publisher<std_msgs::msg::String>("/UR5/cloud_messages", 10);
        
       
        // Create service client
        client_ = this->create_client<common_services_package::srv::GetPlantpotCoords>("/UR5/service/plantpot_coords");
        
        clear_octomap_client_ = this->create_client<std_srvs::srv::Empty>("/occupancy_map_node/clear_octomap");
        
        // Initialize MoveIt interface
        //auto timer = this->create_wall_timer(
        //    std::chrono::seconds(2),
        //    std::bind(&SensorProbeNode::initializeMoveGroup, this));
        
        // Start main processing task
        start_processing_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&SensorProbeNode::startProcessing, this));
    }
    
private:
    
    //void initializeMoveGroup() {
    //    // This method will be called after construction when shared_from_this() is valid
    //    moveit::planning_interface::MoveGroupInterface::Options options("ur5_arm", "robot_description", "/UR5");
    //    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    //        shared_from_this(), 
    //        options);
    //                
    //    move_group_interface_->setPlanningTime(20.0);
    //    move_group_interface_->setMaxVelocityScalingFactor(0.1);
    //    move_group_interface_->setMaxAccelerationScalingFactor(0.1);
    //    move_group_interface_->setPlannerId("BiTRRT");
    //    move_group_interface_->setNumPlanningAttempts(20);
    //    move_group_interface_->setGoalJointTolerance(0.03);
    //    // Cancel the timer since we only need to run this once
    //    timer_->cancel();
    //}





    // Update your callback functions
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(camera_data_mutex_);
    camera_info_ = msg;
    new_camera_info_ = true;
    }

    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(camera_data_mutex_);
    depth_msg_ = msg;
    new_depth_data_ = true;
    }

    void rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(camera_data_mutex_);
    rgb_msg_ = msg;
    new_rgb_data_ = true;
    }
    
    void startProcessing() {
        // Only run once and then cancel timer
        start_processing_timer_->cancel();
        
        if (!camera_info_ || !depth_msg_ || !rgb_msg_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for camera data...");
            start_processing_timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&SensorProbeNode::startProcessing, this));
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Starting main processing task");
        runMain();
    }


   void callClearOctomapService()
   {
    // Check if the service is available
    if (!clear_octomap_client_->wait_for_service(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(this->get_logger(), "Clear octomap service not available");
      return;
    }

    // Create an empty request
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    
    // Call the service asynchronously
    auto future = clear_octomap_client_->async_send_request(request,
      [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
        // This callback gets called when the service call completes
        RCLCPP_INFO(this->get_logger(), "Clear octomap service call completed");
      });
    
    RCLCPP_INFO(this->get_logger(), "Clear octomap service call sent");
   }

    // Wait for new Data
    bool waitForFreshCameraData(double timeout_seconds = 5.0) {
    auto start_time = this->now();
    
    // Reset flags
    {
        std::lock_guard<std::mutex> lock(camera_data_mutex_);
        new_rgb_data_ = false;
        new_depth_data_ = false;
        new_camera_info_ = false;
    }
    
    // Wait until timeout or all data received
    while (rclcpp::ok()) {
        {
        std::lock_guard<std::mutex> lock(camera_data_mutex_);
        if (new_rgb_data_ && new_depth_data_ && new_camera_info_) {
            return true;
        }
        }
        
        // Check for timeout
        auto current_time = this->now();
        if ((current_time - start_time).seconds() > timeout_seconds) {
        RCLCPP_ERROR(this->get_logger(), "Camera Data Time Out");
        return false;
        }
        
        // Sleep briefly to allow callbacks to be processed by the main executor
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return false;
    }
    
    void runMain() {
        // Process point cloud
        auto [filtered_pointcloud, filtered_image] = filterGreenPointCloud(rgb_msg_, depth_msg_, camera_info_);
        
        if (!filtered_pointcloud) {
            RCLCPP_ERROR(this->get_logger(), "Failed to filter point cloud");
            return;
        }
        
        // Publish filtered image
        if (filtered_image) {
            image_pub_->publish(*filtered_image);
        }
        
        // Transform point cloud to world coordinates
        try {
            callClearOctomapService();
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer_->lookupTransform("world", "UR5_camera_color_optical_frame", rclcpp::Time(0), std::chrono::seconds(1));
                
            // Convert to Eigen::Matrix4f
            Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform_stamped.transform);
            Eigen::Matrix4f camera_to_world = transform_eigen.matrix().cast<float>();
            
            RCLCPP_INFO(this->get_logger(), "Got transform from camera to world");
            
            // Transform filtered cloud from camera frame to world frame
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::transformPointCloud(*filtered_pointcloud, *cloud_world, camera_to_world);
            
            // Find highest point in the cloud
            float max_height = -std::numeric_limits<float>::max();
            for (const auto& point : cloud_world->points) {
                if (point.y <= 1.0){
                    if (point.z > max_height) {
                        max_height = point.z;
                    }
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "Maximum height found: %.3f", max_height);


            if (filtered_pointcloud->empty()) {
             RCLCPP_WARN(this->get_logger(), "Filtered point cloud is empty, not publishing");
            }
            // Publish transformed point cloud
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*filtered_pointcloud, cloud_msg);
            cloud_msg.header.frame_id = "UR5_camera_color_optical_frame";
            cloud_msg.header.stamp = this->now();
            pointcloud_pub_->publish(cloud_msg);
            // Wait for the service to be available
            if (!client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
                return;
            }
            
            // Create a request
            auto request = std::make_shared<common_services_package::srv::GetPlantpotCoords::Request>();
            
            // Send the request
            auto result_future = client_->async_send_request(request,
                [this, max_height](rclcpp::Client<common_services_package::srv::GetPlantpotCoords>::SharedFuture future) {
                    auto msg = future.get()->coordinates;
                    RCLCPP_INFO(this->get_logger(), "Received point: x=%.3f, y=%.3f, z=%.3f", msg.x, msg.y, msg.z);
                    processTargetCoordinates(msg, max_height);
                });
                
        } catch (const tf2::TransformException& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", e.what());
        }
    }
    
    void processTargetCoordinates(const geometry_msgs::msg::Point& msg, float max_height) {
        std::vector<geometry_msgs::msg::Pose> target_poses;
        
        // Center of the semicircle
        double center_x = msg.x;
        double center_y = msg.y;
        double center_z = 0 + 0.20;

        // Define the radius of the semicircle
        const double radius = 0.3;
        const int num_positions = 5;

        //calculation Point nearest (0,0,z)
        double start_angle = atan2(center_y, center_x) + M_PI;

        double Angles[5] = {start_angle - M_PI/4 , start_angle , start_angle + M_PI/4 , start_angle + M_PI/2, start_angle + M_PI/4 + M_PI/2};
        
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
            target_pose.position.z = max_height + 0.1;

            double vx = center_x - x;
            double vy = center_y - y;
            double vz = center_z - max_height - 0.1;

            double Norm = sqrt(vx*vx + vy*vy + vz*vz);

            // Step 3: Define the reference vector (assuming robot arm points along Z-axis)
            double rx = 0, ry = 0, rz = 1; // World Z-axis

            // Step 4: Compute the cross product (axis of rotation)
            double axis_x = ry * vz/Norm  - rz * vy/Norm;
            double axis_y = rz * vx/Norm - rx * vz/Norm;
            double axis_z = rx * vy/Norm - ry * vx/Norm;

            double dotProduct = rx * vx/Norm + ry * vy/Norm + rz * vz/Norm;
            double angle = acos(dotProduct); // Angle between the two vectors

            // Normalize the rotation axis
            double norm = sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z);
            if (norm > 0.0) {
                axis_x /= norm;
                axis_y /= norm;
                axis_z /= norm;
            }

            tf2::Quaternion arm_orientation;
            arm_orientation.setX(axis_x * sin(angle / 2));
            arm_orientation.setY(axis_y * sin(angle / 2));
            arm_orientation.setZ(axis_z * sin(angle / 2));
            arm_orientation.setW(cos(angle / 2));

            double angle_to_rotate_z = atan2(vy, vx);

            // Define the fixed camera rotation quaternion
            tf2::Quaternion camera_rotation;
            camera_rotation.setRPY(0.0, 0.0, angle_to_rotate_z + M_PI/2);

            // Combine the two quaternions
            tf2::Quaternion final_orientation = arm_orientation * camera_rotation;
            final_orientation.normalize();

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
            marker.header.stamp = this->now();
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
            marker_pub_->publish(marker);
            
/*             geometry_msgs::msg::Pose above_target_pose;
            above_target_pose = target_pose;
            above_target_pose.position.z = center_z + 0.07;
            
            // Create a marker to visualize the target position in RViz
            visualization_msgs::msg::Marker marker_top;
            marker_top.header.frame_id = "world";
            marker_top.header.stamp = this->now();
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
            marker_top.color.a = 0.7;
            
            // Publish the marker
            marker_pub_->publish(marker_top);
            
            target_poses.push_back(above_target_pose); */
            target_poses.push_back(target_pose);
        }
        std::this_thread::sleep_for(std::chrono::seconds(5)); 
        RCLCPP_INFO(this->get_logger(), "Positions calculated, moving to target positions...");
        moveToTargets(target_poses);
    }
        
    void moveToTargets(const std::vector<geometry_msgs::msg::Pose>& target_poses) {
        // Launch motion planning and execution in a separate thread
        std::thread motion_thread([this, target_poses]() {
            try {
                using moveit::planning_interface::MoveGroupInterface;
                moveit::planning_interface::MoveGroupInterface::Options options(
                    "ur5_arm", "robot_description", "/UR5");
                auto move_group_interface = MoveGroupInterface(shared_from_this(), options);
                
                // Your existing setup code
                move_group_interface.setPlanningTime(20.0);
                move_group_interface.setMaxVelocityScalingFactor(0.1);
                move_group_interface.setMaxAccelerationScalingFactor(0.1);
                move_group_interface.setPlannerId("BiTRRT");
                move_group_interface.setNumPlanningAttempts(20);
                move_group_interface.setGoalJointTolerance(0.03);

                namespace rvt = rviz_visual_tools;
                moveit_visual_tools::MoveItVisualTools visual_tools(
                    shared_from_this(), "world", "/UR5/move_group_tutorial",
                    move_group_interface.getRobotModel());
                visual_tools.deleteAllMarkers();

                Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
                text_pose.translation().z() = 1.0;
                visual_tools.publishText(text_pose, "Creating_Occupancy_Map", rvt::WHITE,
                                        rvt::XLARGE);
                visual_tools.trigger();

                const moveit::core::RobotModelConstPtr& robot_model = move_group_interface.getRobotModel();
                const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("ur5_arm");

                
                // Add your existing constraints code here
                moveit_msgs::msg::JointConstraint shoulder_pan_constraint;
                shoulder_pan_constraint.joint_name = "UR5_shoulder_pan_joint";
                shoulder_pan_constraint.position = M_PI / 2;
                shoulder_pan_constraint.tolerance_above = M_PI;
                shoulder_pan_constraint.tolerance_below = M_PI;
                shoulder_pan_constraint.weight = 0.8;
                
                // Add other constraints...
                moveit_msgs::msg::JointConstraint wrist_1_constraint;
                wrist_1_constraint.joint_name = "UR5_wrist_1_joint";
                wrist_1_constraint.position = 0.0;
                wrist_1_constraint.tolerance_above = M_PI/2;
                wrist_1_constraint.tolerance_below = M_PI/2;
                wrist_1_constraint.weight = 0.8;

                moveit_msgs::msg::JointConstraint wrist_2_constraint;
                wrist_2_constraint.joint_name = "UR5_wrist_2_joint";
                wrist_2_constraint.position = M_PI/2;
                wrist_2_constraint.tolerance_above = M_PI;
                wrist_2_constraint.tolerance_below = M_PI;
                wrist_2_constraint.weight = 0.8;

                moveit_msgs::msg::JointConstraint wrist_3_constraint;
                wrist_3_constraint.joint_name = "UR5_wrist_3_joint";
                wrist_3_constraint.position = 0.0;
                wrist_3_constraint.tolerance_above = M_PI;
                wrist_3_constraint.tolerance_below = 0;
                wrist_3_constraint.weight = 0.8;
                
                RCLCPP_INFO(this->get_logger(), "Motion planning to target %lu", target_poses.size());
                moveit_msgs::msg::Constraints constraints;
                constraints.joint_constraints.push_back(shoulder_pan_constraint);
                constraints.joint_constraints.push_back(wrist_1_constraint);
                constraints.joint_constraints.push_back(wrist_2_constraint);
                constraints.joint_constraints.push_back(wrist_3_constraint);
                move_group_interface.setPathConstraints(constraints);
                
                // Execute motion to each target pose
                bool success;
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                moveit::planning_interface::MoveGroupInterface::Plan plan_to_camera;

                std::vector<double> target_joint_degrees = {90.0, -156.0, 103.0, 80.0, 91.0, 0.0};  // Example joint values in degrees
                std::vector<double> radians;
                for (double degree : target_joint_degrees) {
                        radians.push_back(degree * M_PI / 180.0);  // Convert to radians
                    }
                
                for (size_t i = 0; i < target_poses.size(); i++) {
                    RCLCPP_INFO(this->get_logger(), "Motion planning to target %zu", i);
                    move_group_interface.setPoseTarget(target_poses[i]);

                    visual_tools.deleteAllMarkers();
                    visual_tools.publishText(text_pose, "Calculating_Path_to_Pose_" + std::to_string(i + 1) , rvt::WHITE, rvt::XLARGE);
                    visual_tools.trigger();

                    success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                    if (!success) {
                        RCLCPP_WARN(this->get_logger(), "Planning failed for target %zu", i);
                        continue;
                    }

                    visual_tools.deleteAllMarkers();
                    RCLCPP_INFO(this->get_logger(), "Visualizing Pose trajectory line");
                    visual_tools.publishText(text_pose, "Moving_to_Pose_" + std::to_string(i + 1), rvt::WHITE, rvt::XLARGE);
                    visual_tools.publishTrajectoryLine(plan.trajectory, joint_model_group);
                    visual_tools.trigger();


                    RCLCPP_INFO(this->get_logger(), "Moving to target %zu", i);
                    move_group_interface.execute(plan);
                    
                    RCLCPP_INFO(this->get_logger(), "Processing point cloud at target %zu", i);
                    
                    // Wait for fresh camera data
                    if (waitForFreshCameraData(5.0)) {
                        std::lock_guard<std::mutex> lock(camera_data_mutex_);
                        if (rgb_msg_ && depth_msg_ && camera_info_) {
                            auto [filtered_pc, filtered_img] = filterGreenPointCloud(rgb_msg_, depth_msg_, camera_info_);
                            
                            if (filtered_pc) {
                                // Publish point cloud
                                sensor_msgs::msg::PointCloud2 cloud_msg;
                                pcl::toROSMsg(*filtered_pc, cloud_msg);
                                cloud_msg.header.frame_id = "UR5_camera_color_optical_frame";
                                cloud_msg.header.stamp = this->now();
                                pointcloud_pub_->publish(cloud_msg);
                                RCLCPP_INFO(this->get_logger(), "Published point cloud at target %zu", i);
                                std::this_thread::sleep_for(std::chrono::seconds(2)); 
                            }
                            
                            if (filtered_img) {
                                image_pub_->publish(*filtered_img);
                            }
                        }
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Failed to get fresh camera data at target %zu", i);
                    }
                    move_group_interface.setJointValueTarget(radians);
                    visual_tools.deleteAllMarkers();
                    visual_tools.publishText(text_pose, "Calculating_Path_to_Camera" , rvt::WHITE, rvt::XLARGE);
                    visual_tools.trigger();

                    success = (move_group_interface.plan(plan_to_camera) == moveit::core::MoveItErrorCode::SUCCESS);

                    visual_tools.deleteAllMarkers();
                    RCLCPP_INFO(this->get_logger(), "Visualizing Camera trajectory line");
                    visual_tools.publishText(text_pose, "Moving_back_to_Camera", rvt::WHITE, rvt::XLARGE);
                    visual_tools.publishTrajectoryLine(plan_to_camera.trajectory, joint_model_group);
                    visual_tools.trigger();

                    move_group_interface.execute(plan_to_camera);
                }
                
                // Return to home position
                RCLCPP_INFO(this->get_logger(), "Moving to home position");
                move_group_interface.setNamedTarget("Home");
                visual_tools.deleteAllMarkers();
                visual_tools.publishText(text_pose, "Calculating_Path_to_Home" , rvt::WHITE, rvt::XLARGE);
                visual_tools.trigger();

                success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (success) {
                    visual_tools.deleteAllMarkers();
                    RCLCPP_INFO(this->get_logger(), "Visualizing Camera trajectory line");
                    visual_tools.publishText(text_pose, "Moving_to_Home", rvt::WHITE, rvt::XLARGE);
                    visual_tools.publishTrajectoryLine(plan.trajectory, joint_model_group);
                    visual_tools.trigger();

                    move_group_interface.execute(plan);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to plan motion to home position");
                }
                
                // Signal completion
                RCLCPP_INFO(this->get_logger(), "Motion sequence completed");
                visual_tools.deleteAllMarkers();
                visual_tools.trigger();
                std_msgs::msg::String msg;
  	            msg.data = "Octomap_Created";
  	            cloud_message_publisher_->publish(msg);     
                std::this_thread::sleep_for(std::chrono::seconds(1));

                rclcpp::shutdown();
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception during motion: %s", e.what());
                rclcpp::shutdown();
            }
        });
        
        // Detach the thread so it can continue running
        motion_thread.detach();
    }
    
    // TF2 objects
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cloud_message_publisher_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    
    // Messages
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
    sensor_msgs::msg::Image::SharedPtr depth_msg_;
    sensor_msgs::msg::Image::SharedPtr rgb_msg_;

    // new Camera Data Checks
    std::mutex camera_data_mutex_;
    bool new_rgb_data_ = false;
    bool new_depth_data_ = false;
    bool new_camera_info_ = false;
    
    // Service client
    rclcpp::Client<common_services_package::srv::GetPlantpotCoords>::SharedPtr client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_octomap_client_;
    
    // MoveIt interface
    //std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    //rclcpp::TimerBase::SharedPtr timer_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr start_processing_timer_;
};

int main(int argc, char **argv) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorProbeNode>();
    
    // Create a multithreaded executor to handle callbacks
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);  // Use 2 threads
    executor.add_node(node);
    
    // Spin and process callbacks
    executor.spin();
    
    // Shutdown ROS after the task is finished
    rclcpp::shutdown();
    return 0;
}
