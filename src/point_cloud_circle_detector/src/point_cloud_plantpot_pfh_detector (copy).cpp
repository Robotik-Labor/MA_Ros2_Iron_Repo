#define PCL_NO_PRECOMPILE
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
            "/UR5/UR5_camera/depth/color/points", 1,
            std::bind(&PointCloudCustomObjectMatcher::listener_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PointCloud Custom Object Matcher Node has been started.");

        // Load the custom object PCD (change the path accordingly)
        std::string pcd_path = "/home/buhrmann/ws_moveit/install/point_cloud_circle_detector/share/point_cloud_circle_detector/pcd/plantpod_lesssamples.pcd";
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
	    start_time = std::chrono::high_resolution_clock::now(); // Start timing
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	    filter_by_rgb(cloud, cloud_rgb_filtered);
	    end_time = std::chrono::high_resolution_clock::now(); // End timing
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "RGB filtering took %f ms", duration.count());
	    
	    
	        	    
	    // Publish the filtered point cloud just before matching
	    sensor_msgs::msg::PointCloud2 output_pre_segmentation;
	    pcl::toROSMsg(*cloud_rgb_filtered, output_pre_segmentation);
	    output_pre_segmentation.header.stamp = this->get_clock()->now();
	    output_pre_segmentation.header.frame_id = "UR5_camera_depth_optical_frame"; // Ensure this matches your RViz fixed frame
	    pre_segmentation_publisher_->publish(output_pre_segmentation);
	    
	    

	    // Step 3: Convert PointCloudXYZRGB to PointCloudXYZ
	    start_time = std::chrono::high_resolution_clock::now(); // Start timing
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	    for (const auto& point : cloud_rgb_filtered->points)
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
	    voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f);
	    voxel_grid.filter(*cloud_filtered_voxel);
	    end_time = std::chrono::high_resolution_clock::now(); // End timing
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "Voxel grid filtering took %f ms", duration.count());

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


	    // Step 6: Compute PPF features for the target (scene) point cloud
	    start_time = std::chrono::high_resolution_clock::now(); // Start timing
	    pcl::PointCloud<pcl::PFHSignature125>::Ptr cloud_pfh_features(new pcl::PointCloud<pcl::PFHSignature125>);
	    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_estimator;
	    pfh_estimator.setInputCloud(cloud_filtered_voxel);  // Set input cloud
	    pfh_estimator.setInputNormals(normals);  // Set input normals
	    pfh_estimator.setRadiusSearch(0.03);
	    pfh_estimator.compute(*cloud_pfh_features);  // Compute PPF features
	    end_time = std::chrono::high_resolution_clock::now(); // End timing
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "PPF feature extraction for scene took %f ms", duration.count());
	    
	    
	    
	    // Step 5: Apply voxel grid filter for downsampling
	    start_time = std::chrono::high_resolution_clock::now(); // Start timing
	    pcl::PointCloud<pcl::PointXYZ>::Ptr object_voxel(new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::VoxelGrid<pcl::PointXYZ> voxel_object_grid;
	    voxel_object_grid.setInputCloud(custom_object_);
	    voxel_object_grid.setLeafSize(0.05f, 0.05f, 0.05f);
	    voxel_object_grid.filter(*object_voxel);
	    end_time = std::chrono::high_resolution_clock::now(); // End timing
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "Voxel grid filtering took %f ms", duration.count());

	    // Step 7: Compute PFH features for the source (custom object) point cloud
	    start_time = std::chrono::high_resolution_clock::now(); // Start timing
	    pcl::PointCloud<pcl::Normal>::Ptr custom_object_normals(new pcl::PointCloud<pcl::Normal>);
	    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_custom;
	    ne_custom.setInputCloud(object_voxel);
	    ne_custom.setSearchMethod(tree);
	    ne_custom.setRadiusSearch(0.03);  // Adjust accordingly
	    ne_custom.compute(*custom_object_normals);

	    pcl::PointCloud<pcl::PFHSignature125>::Ptr custom_object_pfh_features(new pcl::PointCloud<pcl::PFHSignature125>);
	    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_custom;
	    pfh_custom.setInputCloud(object_voxel);
	    pfh_custom.setInputNormals(custom_object_normals);
	    pfh_custom.setRadiusSearch(0.03);
	    pfh_custom.compute(*custom_object_pfh_features);
	    end_time = std::chrono::high_resolution_clock::now(); // End timing
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "PPF feature extraction for custom object took %f ms", duration.count());
	    
	    RCLCPP_INFO(this->get_logger(), "Source points points: %zu", object_voxel->points.size());
	    RCLCPP_INFO(this->get_logger(), "Target points points: %zu", cloud_filtered_voxel->points.size());

	    
	    RCLCPP_INFO(this->get_logger(), "Source feature points: %zu", custom_object_pfh_features->points.size());
	    RCLCPP_INFO(this->get_logger(), "Target feature points: %zu", cloud_pfh_features->points.size());


	    // Step 8: RANSAC-based Model Alignment (SAC-IA) to detect the pod in the scene
	    start_time = std::chrono::high_resolution_clock::now(); // Start timing
	    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::PFHSignature125> sac_ia;
	    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_object(new pcl::PointCloud<pcl::PointXYZ>);

	    // Setup the RANSAC algorithm
	    sac_ia.setInputSource(object_voxel);  // The pod object model
	    sac_ia.setInputTarget(cloud_filtered_voxel);  // The filtered scene point cloud
	    sac_ia.setSourceFeatures(custom_object_pfh_features);  // Set the PPF features for the source
	    sac_ia.setTargetFeatures(cloud_pfh_features);  // Set the PPF features for the target
	    sac_ia.setMaxCorrespondenceDistance(0.05);  // Max distance between correspondences
	    sac_ia.setMaximumIterations(10000);  // Max iterations to run RANSAC
	    sac_ia.setMinSampleDistance(0.01);  // Minimum sample distance
	    sac_ia.setNumberOfSamples(3);  // Minimum number of samples to define a transformation

	    // Apply the RANSAC-based alignment
	    sac_ia.align(*aligned_object);
	    end_time = std::chrono::high_resolution_clock::now(); // End timing
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "RANSAC alignment took %f ms", duration.count());

	    if (sac_ia.hasConverged())
	    {
		RCLCPP_INFO(this->get_logger(), "SAC-IA has converged, transformation found.");
		    Eigen::Matrix4f final_transformation = sac_ia.getFinalTransformation();

		    // Print the full transformation matrix (translation and rotation)
		    std::cout << "Final Transformation Matrix: \n" << final_transformation << std::endl;

			// Extract the translation vector (last column of the transformation matrix)
			Eigen::Vector3f translation_only = final_transformation.block<3, 1>(0, 3);  // Extract translation (x, y, z)
			translation_only.y() = 0.0f;

			// Print the translation-only vector
			std::cout << "Translation Vector: \n" << translation_only.transpose() << std::endl;

			// Create a 4x4 transformation matrix with the translation (and no rotation)
			Eigen::Matrix4f translation_matrix = Eigen::Matrix4f::Identity();
			translation_matrix.block<3, 1>(0, 3) = translation_only;  // Set the translation part of the matrix

			// Apply the translation matrix to the source point cloud (no rotation)
			pcl::PointCloud<pcl::PointXYZ>::Ptr translated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::transformPointCloud(*custom_object_, *translated_cloud, translation_matrix);

		    // Publish the translated cloud (without rotation)
		    sensor_msgs::msg::PointCloud2 output_aligned;
		    pcl::toROSMsg(*translated_cloud, output_aligned);
		    output_aligned.header.stamp = this->get_clock()->now();
		    output_aligned.header.frame_id = "UR5_camera_depth_optical_frame";  // Specify the frame of reference
		    point_cloud_publisher_->publish(output_aligned);
	    }
	    else
	    {
		RCLCPP_WARN(this->get_logger(), "SAC-IA alignment did not converge.");
	    }
	}


    void filter_by_rgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud)
    {
        output_cloud->clear();

        for (const auto& point : input_cloud->points)
        {
            if (point.r >= 105 && point.r <= 135 && point.g >= 44 && point.g <= 74 && point.b >= 22 && point.b <= 52)
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

