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
  
	        	   
	    // Step 2: Convert PointCloudXYZRGB to PointCloudXYZ
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
	    
	    // Step 5: Apply voxel grid filter for downsampling
	    start_time = std::chrono::high_resolution_clock::now(); // Start timing
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	    voxel_grid.setInputCloud(cloud_filtered_xyz);
	    voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);
	    voxel_grid.filter(*cloud_filtered_voxel);
	    end_time = std::chrono::high_resolution_clock::now(); // End timing
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "Voxel grid filtering took %f ms", duration.count());
	    
   	    
	    // Step 3: Publish the filtered point cloud just before matching
	    //sensor_msgs::msg::PointCloud2 output_pre_segmentation;
	    //pcl::toROSMsg(*cloud_filtered_voxel, output_pre_segmentation);
	    //output_pre_segmentation.header.stamp = this->get_clock()->now();
	    //output_pre_segmentation.header.frame_id = "UR5_camera_depth_optical_frame"; // Ensure this matches your RViz fixed frame
	    //pre_segmentation_publisher_->publish(output_pre_segmentation);
	    


	    // Step 4: Compute normals for the filtered point cloud
	    start_time = std::chrono::high_resolution_clock::now(); // Start timing
	    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	    ne.setInputCloud(cloud_filtered_voxel);
	    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	    ne.setSearchMethod(tree);
	    ne.setRadiusSearch(0.03); // Adjust the radius based on your point cloud density
	    ne.compute(*normals);
	    end_time = std::chrono::high_resolution_clock::now(); // End timing
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "Normal computation took %f ms", duration.count());

	    // Step 5: Compute PPF features for the scene point cloud
	    start_time = std::chrono::high_resolution_clock::now(); // Start timing
	    pcl::PointCloud<pcl::PFHSignature125>::Ptr cloud_pfh_features(new pcl::PointCloud<pcl::PFHSignature125>);
	    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_estimator;
	    pfh_estimator.setInputCloud(cloud_filtered_voxel);  // Set input cloud
	    pfh_estimator.setInputNormals(normals);  // Set input normals
	    pfh_estimator.setRadiusSearch(0.03);
	    pfh_estimator.compute(*cloud_pfh_features);  // Compute PPF features
	    end_time = std::chrono::high_resolution_clock::now(); // End timing
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "PFH feature extraction for scene took %f ms", duration.count());
	    
	    // Step 6: Apply voxel grid filter to custom_object for downsampling
	    start_time = std::chrono::high_resolution_clock::now(); // Start timing
	    pcl::PointCloud<pcl::PointXYZ>::Ptr object_voxel(new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::VoxelGrid<pcl::PointXYZ> voxel_object_grid;
	    voxel_object_grid.setInputCloud(custom_object_);
	    voxel_object_grid.setLeafSize(0.01f, 0.01f, 0.01f);
	    voxel_object_grid.filter(*object_voxel);
	    end_time = std::chrono::high_resolution_clock::now(); // End timing
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "Voxel grid filtering took %f ms", duration.count());

	    // Step 7: Compute normals for the custom object pointcloud 
	    start_time = std::chrono::high_resolution_clock::now(); // Start timing
	    pcl::PointCloud<pcl::Normal>::Ptr custom_object_normals(new pcl::PointCloud<pcl::Normal>);
	    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_custom;
	    ne_custom.setInputCloud(object_voxel);
	    ne_custom.setSearchMethod(tree);
	    ne_custom.setRadiusSearch(0.03);  // Adjust accordingly
	    ne_custom.compute(*custom_object_normals);
	    
	    
	
	    // Step 8: Compute PFH features for the custom object pointcloud
	    pcl::PointCloud<pcl::PFHSignature125>::Ptr custom_object_pfh_features(new pcl::PointCloud<pcl::PFHSignature125>);
	    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_custom;
	    pfh_custom.setInputCloud(object_voxel);
	    pfh_custom.setInputNormals(custom_object_normals);
	    pfh_custom.setRadiusSearch(0.03);
	    pfh_custom.compute(*custom_object_pfh_features);
	    end_time = std::chrono::high_resolution_clock::now(); // End timing
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "PFH feature extraction for custom object took %f ms", duration.count());
	    
	    RCLCPP_INFO(this->get_logger(), "Object points points: %zu", object_voxel->points.size());
	    RCLCPP_INFO(this->get_logger(), "Target points points: %zu", cloud_filtered_voxel->points.size());

	    
	    RCLCPP_INFO(this->get_logger(), "Object feature points: %zu", custom_object_pfh_features->points.size());
	    RCLCPP_INFO(this->get_logger(), "Target feature points: %zu", cloud_pfh_features->points.size());

	    // Step 9: RANSAC-based Model Alignment (SAC-IA) to detect the pod in the scene
	    
	        Eigen::Matrix4f camera_to_world = getCameraToWorldTransform();
		    // Transform filtered cloud from camera frame to world frame
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_world(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::transformPointCloud(*cloud_filtered_voxel, *cloud_filtered_world, camera_to_world);
		
		Eigen::Vector4f scene_centroid;
		pcl::compute3DCentroid(*cloud_filtered_world, scene_centroid);
	    	
	    	pcl::PointCloud<pcl::PointXYZ>::Ptr centered_cloud_filtered_world(new pcl::PointCloud<pcl::PointXYZ>());
	    	centered_cloud_filtered_world->header.frame_id = "world";  // Setting the frame of the destination cloud to "world"
	        // Iterate through the source cloud, modify the points, and copy them to the destination cloud
	        centered_cloud_filtered_world->width = cloud_filtered_world->width;  // Same width as source cloud
	        centered_cloud_filtered_world->height = cloud_filtered_world->height;  // Same height as source cloud
	        centered_cloud_filtered_world->points.resize(centered_cloud_filtered_world->width * centered_cloud_filtered_world->height);
	    	
	    	for (size_t i = 0; i < cloud_filtered_world->points.size(); ++i){
	    	centered_cloud_filtered_world->points[i].x = cloud_filtered_world->points[i].x - scene_centroid[0];
		centered_cloud_filtered_world->points[i].y = cloud_filtered_world->points[i].y - scene_centroid[1];
		centered_cloud_filtered_world->points[i].z = cloud_filtered_world->points[i].z - scene_centroid[2];  
		}
		
		object_voxel->header.frame_id = "world";
		
		sensor_msgs::msg::PointCloud2 output_pre_segmentation;
	    	pcl::toROSMsg(*cloud_filtered_voxel, output_pre_segmentation);
	    	output_pre_segmentation.header.stamp = this->get_clock()->now();
	    	output_pre_segmentation.header.frame_id = "UR5_camera_depth_optical_frame"; // Ensure this matches your RViz fixed frame
	    	pre_segmentation_publisher_->publish(output_pre_segmentation);
	    	
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		
		

		// Set the input source and target
		icp.setInputSource(object_voxel);  // Your object after SAC-IA transform
		icp.setInputTarget(cloud_filtered_voxel);  // Your scene point cloud

		// Set ICP parameters
		icp.setMaximumIterations(100);  // Max iterations
		icp.setTransformationEpsilon(1e-8);  // Transformation epsilon
		icp.setMaxCorrespondenceDistance(0.01);  // Max distance between points
		icp.setEuclideanFitnessEpsilon(0.3);  // Convergence condition
		icp.setRANSACOutlierRejectionThreshold(0.05);  // RANSAC outlier rejection threshold

		// Run ICP
		pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		icp.align(*aligned_cloud);

		// Check if ICP converged and get the fitness score
		if (icp.hasConverged()) {
		    std::cout << "ICP converged with fitness score: " << icp.getFitnessScore() << std::endl;
		    
		    // Get the final transformation matrix
		    Eigen::Matrix4f transformation = icp.getFinalTransformation();
		    std::cout << "Final transformation: " << std::endl << transformation << std::endl;
		    
		    
		sensor_msgs::msg::PointCloud2 output_msg;
		pcl::toROSMsg(*aligned_cloud, output_msg);

		// Set the header for the PointCloud2 message
		output_msg.header.stamp = this->get_clock()->now();
		output_msg.header.frame_id = "world";  // Ensure this matches your desired frame

		// Publish the PointCloud2 message
		point_cloud_publisher_->publish(output_msg);
			    
		    
		} else {
		    std::cout << "ICP did not converge" << std::endl;
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

