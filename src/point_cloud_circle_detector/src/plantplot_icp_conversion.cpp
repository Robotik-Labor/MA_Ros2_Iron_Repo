#define PCL_NO_PRECOMPILE
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfh_tools.h>
#include <pcl/filters/voxel_grid.h>  // Include Voxel Grid filter
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS2 Publisher for PointCloud2
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pre_segmentation_publisher_;  // New publisher for pre-segmentation point cloud
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr custom_object_publisher_;  // New publisher for custom object
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


// Define a constant to toggle the orientation transformation
const bool apply_orientation_override = false;  // Set to false to disable


class PointCloudCustomObjectMatcher : public rclcpp::Node
{
public:
    PointCloudCustomObjectMatcher()
        : Node("point_cloud_custom_object_matcher")
    {	
    
        
        // Publisher to publish PointCloud2 data with Reliable QoS
        point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/UR5/detected_object_point_cloud", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

        // Publisher for the pre-segmentation (filtered) point cloud
        pre_segmentation_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/UR5/pre_segmentation_point_cloud", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

        // Publisher for custom object at the world frame
        custom_object_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/UR5/custom_object_point_cloud", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

        // Create subscription to PointCloud2
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/UR5/plantpod_reduced_pointcloud", 1,
            std::bind(&PointCloudCustomObjectMatcher::listener_callback, this, std::placeholders::_1));

	// Initialize in constructor or init function
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "PointCloud Custom Object Matcher Node has been started.");

        // Load the custom object PCD (change the path accordingly)
        std::string pcd_path = "/home/buhrmann/ws_moveit/install/point_cloud_circle_detector/share/point_cloud_circle_detector/pcd/plantpod_14.pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *object_cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't load the custom object PCD file.");
        }
        custom_object_ = object_cloud;

        // Publish custom object at the world frame (center)
        publish_custom_object_to_world();
    }

private:

Eigen::Matrix4f getCameraToWorldTransform()
{
    try
    {
        // Look up transform from camera frame to world frame
        geometry_msgs::msg::TransformStamped transform_stamped = 
            tf_buffer_->lookupTransform("world", "UR5_camera_depth_optical_frame", rclcpp::Time(0), std::chrono::seconds(1));
        
        // Convert to Eigen::Matrix4f
        Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform_stamped.transform);
        Eigen::Matrix4f camera_to_world = transform_eigen.matrix().cast<float>();
        
        RCLCPP_INFO(this->get_logger(), "Got transform from camera to world");
        return camera_to_world;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not transform from %s to %s: %s",
                    "UR5_camera_depth_optical_frame", "world", ex.what());
        
        // Return identity matrix in case of failure
        return Eigen::Matrix4f::Identity();
    }
}


void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "PointCloud2 message received");

    // Declare time variables for timing different steps
    auto start_time = std::chrono::high_resolution_clock::now();
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration;

    // Step 1: Convert PointCloud2 message to PCL PointCloud
    start_time = std::chrono::high_resolution_clock::now(); // Start timing
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);
    end_time = std::chrono::high_resolution_clock::now(); // End timing
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(this->get_logger(), "PointCloud2 to PCL conversion took %f ms", duration.count());

    // Step 2: Apply RGB color filtering
    //start_time = std::chrono::high_resolution_clock::now(); // Start timing
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    //filter_by_rgb(cloud, cloud_rgb_filtered);
    //end_time = std::chrono::high_resolution_clock::now(); // End timing
    //duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    //RCLCPP_INFO(this->get_logger(), "RGB filtering took %f ms", duration.count());

    // Step 3: Convert PointCloudXYZRGB to PointCloudXYZ
    start_time = std::chrono::high_resolution_clock::now(); // Start timing
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : cloud->points)
    {
        pcl::PointXYZ point_xyz;
        point_xyz.x = point.x;
        point_xyz.y = point.y;
        point_xyz.z = point.z;
        cloud_filtered_xyz->points.push_back(point_xyz);
    }
    end_time = std::chrono::high_resolution_clock::now(); // End timing
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(this->get_logger(), "Conversion to PointCloudXYZ took %f ms", duration.count());
    
    // Step 4: Apply voxel grid filter for downsampling
    start_time = std::chrono::high_resolution_clock::now(); // Start timing
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    //voxel_grid.setInputCloud(cloud_filtered_xyz);
    //voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);
    //voxel_grid.filter(*cloud_filtered_voxel);
    end_time = std::chrono::high_resolution_clock::now(); // End timing
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(this->get_logger(), "Voxel grid filtering took %f ms", duration.count());
    
    // Publish the filtered point cloud just before matching
    //sensor_msgs::msg::PointCloud2 output_pre_segmentation;
    //pcl::toROSMsg(*cloud_filtered_xyz, output_pre_segmentation);
    //output_pre_segmentation.header.stamp = this->get_clock()->now();
    //output_pre_segmentation.header.frame_id = "UR5_camera_depth_optical_frame"; // Ensure this matches your RViz fixed frame
    //pre_segmentation_publisher_->publish(output_pre_segmentation);
    
    // Get transform from camera to world
    Eigen::Matrix4f camera_to_world = getCameraToWorldTransform();
    // Transform filtered cloud from camera frame to world frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_world(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud_filtered_xyz, *cloud_filtered_world, camera_to_world);
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_algined_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Set the frame_id for the destination cloud
    pointcloud_algined_cloud->header.frame_id = "world";  // Setting the frame of the destination cloud to "world"
    // Iterate through the source cloud, modify the points, and copy them to the destination cloud
    pointcloud_algined_cloud->width = cloud_filtered_world->width;  // Same width as source cloud
    pointcloud_algined_cloud->height = cloud_filtered_world->height;  // Same height as source cloud
    pointcloud_algined_cloud->points.resize(pointcloud_algined_cloud->width * pointcloud_algined_cloud->height);

    for (size_t i = 0; i < cloud_filtered_world->points.size(); ++i){
    	pointcloud_algined_cloud->points[i].x = cloud_filtered_world->points[i].x;  // Keep X as is
        pointcloud_algined_cloud->points[i].y = cloud_filtered_world->points[i].y;  // Swap Y with Z
        pointcloud_algined_cloud->points[i].z = cloud_filtered_world->points[i].z;  // Swap Z with Y and negate
        }

    // Publish the filtered and transformed point cloud just before matching
    sensor_msgs::msg::PointCloud2 output_pre_segmentation;
    pcl::toROSMsg(*cloud_filtered_world, output_pre_segmentation);
    output_pre_segmentation.header.stamp = this->get_clock()->now();
    output_pre_segmentation.header.frame_id = "world"; // Ensure this matches your RViz fixed frame
    pre_segmentation_publisher_->publish(output_pre_segmentation);
    
    

    // Downsample the custom object model
    start_time = std::chrono::high_resolution_clock::now(); // Start timing
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_object_grid;
    voxel_object_grid.setInputCloud(custom_object_);
    voxel_object_grid.setLeafSize(0.01f, 0.01f, 0.01f);
    voxel_object_grid.filter(*object_voxel);
    end_time = std::chrono::high_resolution_clock::now(); // End timing
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(this->get_logger(), "Custom object voxel grid filtering took %f ms", duration.count());

    // Apply an initial transformation to the pot model based on known orientation and height
    start_time = std::chrono::high_resolution_clock::now(); // Start timing
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_initial_transform(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f initial_transform = Eigen::Matrix4f::Identity();
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr custom_object_algined_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Set the frame_id for the destination cloud
    custom_object_algined_cloud->header.frame_id = "world";  // Setting the frame of the destination cloud to "world"
    // Iterate through the source cloud, modify the points, and copy them to the destination cloud
    custom_object_algined_cloud->width = object_voxel->width;  // Same width as source cloud
    custom_object_algined_cloud->height = object_voxel->height;  // Same height as source cloud
    custom_object_algined_cloud->points.resize(custom_object_algined_cloud->width * custom_object_algined_cloud->height);

    for (size_t i = 0; i < object_voxel->points.size(); ++i){
    	custom_object_algined_cloud->points[i].x = object_voxel->points[i].x;  // Keep X as is
        custom_object_algined_cloud->points[i].y = object_voxel->points[i].y;  // Swap Y with Z
        custom_object_algined_cloud->points[i].z = object_voxel->points[i].z;  // Swap Z with Y and negate
        }
    
    
    

    if (apply_orientation_override)
    {
        // Apply known orientation and height position
        // Note: Adjust these values according to your known pot orientation and height
        float height_position = 0.006f;  // Example height in meters
        
        // Set rotation (if known) - this is an example with a 30-degree rotation around Y axis
        Eigen::AngleAxisf rotation_y(M_PI/6, Eigen::Vector3f::UnitY());
        Eigen::Matrix3f rotation_matrix = rotation_y.toRotationMatrix();
        
        // Fill the rotation part of the transformation matrix
        initial_transform.block<3,3>(0,0) = rotation_matrix;
        
        // Set the height position (adjust the axis based on your coordinate system)
        initial_transform(2,3) = height_position;  // Y-axis height adjustment
        
        RCLCPP_INFO(this->get_logger(), "Applied initial orientation and height transformation");
    }
    else
    {
        // If no orientation override, use a simpler initial guess
        // This can be the centroid of the segmented scene
        Eigen::Vector4f scene_centroid;
        pcl::compute3DCentroid(*pointcloud_algined_cloud, scene_centroid);
        
        // Place the model near the centroid of the scene
	initial_transform(0,3) = scene_centroid[0];
	initial_transform(1,3) = scene_centroid[2];
	initial_transform(2,3) = -scene_centroid[1];
        
        RCLCPP_INFO(this->get_logger(), "  Translation: [%f, %f, %f]", 
                    initial_transform(0,3), initial_transform(1,3), initial_transform(2,3));
        
        RCLCPP_INFO(this->get_logger(), "Applied simple centroid-based initial transformation");
    }
    
    // Apply the initial transformation
    pcl::transformPointCloud(*custom_object_algined_cloud, *object_initial_transform, initial_transform);
    end_time = std::chrono::high_resolution_clock::now(); // End timing
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(this->get_logger(), "Initial transformation took %f ms", duration.count());
    
    
            // Publish the aligned object cloud
        //sensor_msgs::msg::PointCloud2 output_aligned;
        //pcl::toROSMsg(*object_initial_transform, output_aligned);
        //output_aligned.header.stamp = this->get_clock()->now();
        //output_aligned.header.frame_id = "world";  // Ensure this matches your camera frame
        //point_cloud_publisher_->publish(output_aligned);

    // Step 5: Apply ICP for precise alignment
    start_time = std::chrono::high_resolution_clock::now(); // Start timing
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_object(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Set ICP parameters
    icp.setInputSource(object_initial_transform);  // The pre-positioned pot model
    icp.setInputTarget(pointcloud_algined_cloud);      // The filtered scene point cloud
    icp.setMaxCorrespondenceDistance(0.03);        // 5cm - adjust based on your scale
    icp.setMaximumIterations(50);                  // Maximum iterations
    icp.setTransformationEpsilon(1e-8);            // Transformation epsilon
    icp.setEuclideanFitnessEpsilon(1e-6);          // Fitness epsilon
    
    // Run ICP alignment
    icp.align(*aligned_object);
    end_time = std::chrono::high_resolution_clock::now(); // End timing
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(this->get_logger(), "ICP alignment took %f ms", duration.count());

    if (icp.hasConverged())
    {
        RCLCPP_INFO(this->get_logger(), "ICP has converged with fitness score: %f", icp.getFitnessScore());
        
        // Get the final transformation matrix (combines initial transform and ICP result)
        Eigen::Matrix4f final_transformation = icp.getFinalTransformation();
        // First move to centroid, then apply ICP refinement
	//Eigen::Matrix4f complete_transform =  initial_transform * final_transformation;
	//pcl::transformPointCloud(*custom_object_, *aligned_object, complete_transform);





        //RCLCPP_INFO(this->get_logger(), "Complete transformation:");
        //RCLCPP_INFO(this->get_logger(), "  Translation: [%f, %f, %f]", 
        //            complete_transform(0,3), complete_transform(1,3), complete_transform(2,3));
        
        // Print transformation details for debugging
        RCLCPP_INFO(this->get_logger(), "Final transformation:");
        RCLCPP_INFO(this->get_logger(), "  Translation: [%f, %f, %f]", 
                    final_transformation(0,3), final_transformation(1,3), final_transformation(2,3));
                    
        // Extract rotation as Euler angles (in radians)
        Eigen::Matrix3f rotation = final_transformation.block<3,3>(0,0);
        Eigen::Vector3f euler_angles = rotation.eulerAngles(0, 1, 2);  // Roll, Pitch, Yaw
        RCLCPP_INFO(this->get_logger(), "  Rotation (RPY): [%f, %f, %f]", 
                    euler_angles[0], euler_angles[1], euler_angles[2]);


    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "ICP alignment did not converge.");
    }  
    	Eigen::Matrix4f final_transformation2 = icp.getFinalTransformation();
        // Apply the complete transformation to the original custom object
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_result(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*custom_object_, *final_result, initial_transform *final_transformation2);
        //Publish the aligned object cloud
        sensor_msgs::msg::PointCloud2 output_aligned;
        pcl::toROSMsg(*final_result, output_aligned);
        output_aligned.header.stamp = this->get_clock()->now();
        output_aligned.header.frame_id = "world";  // Ensure this matches your camera frame
        point_cloud_publisher_->publish(output_aligned);
}


    void filter_by_rgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud)
    {
        output_cloud->clear();

        for (const auto& point : input_cloud->points)
        {
            if (point.r >= 80 && point.r <= 155 && point.g >= 20 && point.g <= 100 && point.b >= 0 && point.b <= 70)
            {
                output_cloud->points.push_back(point);
            }
        }
    }

    void publish_custom_object_to_world()
    {
        if (!custom_object_->empty())
        {
            sensor_msgs::msg::PointCloud2 output_custom_object;
            pcl::toROSMsg(*custom_object_, output_custom_object);
            output_custom_object.header.stamp = this->get_clock()->now();
            output_custom_object.header.frame_id = "world";  // World frame
            custom_object_publisher_->publish(output_custom_object);
        }
    }
    
void overrideOrientationAndPlaceOnPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
}





private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr custom_object_;  // Custom object model
};

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create the node
    rclcpp::spin(std::make_shared<PointCloudCustomObjectMatcher>());

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}

