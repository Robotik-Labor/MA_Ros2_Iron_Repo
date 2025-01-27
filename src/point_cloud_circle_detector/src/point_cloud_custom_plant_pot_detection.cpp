#define PCL_NO_PRECOMPILE
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
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
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>

// ROS2 Publisher for PointCloud2
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pre_segmentation_publisher_;  // New publisher for pre-segmentation point cloud
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr custom_object_publisher_;  // New publisher for custom object

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

	    // Start timing the point cloud conversion
	    auto start_time = std::chrono::high_resolution_clock::now();

	    // Convert PointCloud2 message to PCL PointCloud
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	    pcl::fromROSMsg(*msg, *cloud);

	    // End timing the point cloud conversion
	    auto end_time = std::chrono::high_resolution_clock::now();
	    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "PointCloud2 to PCL conversion took %ld ms", duration.count());

	    // Apply RGB color filtering (filter out points not in the range of (120, 59, 37) ± 15)
	    start_time = std::chrono::high_resolution_clock::now();
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	    filter_by_rgb(cloud, cloud_filtered);
	    end_time = std::chrono::high_resolution_clock::now();
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "RGB filtering took %ld ms", duration.count());

	    // Convert PointXYZRGB to PointXYZ here
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	    start_time = std::chrono::high_resolution_clock::now();
	    for (const auto& point : cloud_filtered->points)
	    {
		pcl::PointXYZ point_xyz;
		point_xyz.x = point.x;
		point_xyz.y = point.y;
		point_xyz.z = point.z;
		cloud_filtered_xyz->points.push_back(point_xyz);
	    }
	    end_time = std::chrono::high_resolution_clock::now();
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "PointXYZ conversion took %ld ms", duration.count());

	    // Apply radius outlier removal filter to clean up the cloud
	    start_time = std::chrono::high_resolution_clock::now();
	    pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
	    sor.setInputCloud(cloud_filtered_xyz);
	    sor.setRadiusSearch(0.01);  // Radius in meters
	    sor.setMinNeighborsInRadius(5);  // Minimum neighbors in radius to be considered a valid point
	    sor.filter(*cloud_filtered_xyz);
	    end_time = std::chrono::high_resolution_clock::now();
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "Radius outlier removal took %ld ms", duration.count());

	    // Compute normals for the target (scene) point cloud
	    start_time = std::chrono::high_resolution_clock::now();
	    pcl::PointCloud<pcl::Normal>::Ptr cloud_filtered_normals(new pcl::PointCloud<pcl::Normal>);
	    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	    ne.setInputCloud(cloud_filtered_xyz);
	    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	    ne.setSearchMethod(tree);
	    ne.setRadiusSearch(0.03);  // Radius for normal estimation
	    ne.compute(*cloud_filtered_normals);
	    end_time = std::chrono::high_resolution_clock::now();
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "Normal estimation took %ld ms", duration.count());

	    // Compute FPFH features for the filtered target point cloud
	    start_time = std::chrono::high_resolution_clock::now();
	    pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_filtered_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
	    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	    fpfh.setInputCloud(cloud_filtered_xyz);
	    fpfh.setInputNormals(cloud_filtered_normals);
	    fpfh.setSearchMethod(tree);
	    fpfh.setRadiusSearch(0.05);
	    fpfh.compute(*cloud_filtered_fpfh);
	    end_time = std::chrono::high_resolution_clock::now();
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "FPFH feature extraction took %ld ms", duration.count());

	    // Compute normals for the source (custom object) point cloud
	    start_time = std::chrono::high_resolution_clock::now();
	    pcl::PointCloud<pcl::Normal>::Ptr custom_object_normals(new pcl::PointCloud<pcl::Normal>);
	    ne.setInputCloud(custom_object_);
	    ne.compute(*custom_object_normals);
	    end_time = std::chrono::high_resolution_clock::now();
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "Custom object normal estimation took %ld ms", duration.count());

	    // Compute FPFH features for the source point cloud
	    start_time = std::chrono::high_resolution_clock::now();
	    pcl::PointCloud<pcl::FPFHSignature33>::Ptr custom_object_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
	    fpfh.setInputCloud(custom_object_);
	    fpfh.setInputNormals(custom_object_normals);
	    fpfh.compute(*custom_object_fpfh);
	    end_time = std::chrono::high_resolution_clock::now();
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "Custom object FPFH feature extraction took %ld ms", duration.count());

	    // Step 1: RANSAC-based Model Alignment (SAC-IA) to detect the pod in the scene
	    start_time = std::chrono::high_resolution_clock::now();
	    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
	    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_object(new pcl::PointCloud<pcl::PointXYZ>);

	    // Setup the RANSAC algorithm
	    sac_ia.setInputSource(custom_object_);  // The pod object model
	    sac_ia.setInputTarget(cloud_filtered_xyz);  // The filtered scene point cloud
	    sac_ia.setSourceFeatures(custom_object_fpfh);  // Set the FPFH features for the source
	    sac_ia.setTargetFeatures(cloud_filtered_fpfh);  // Set the FPFH features for the target
	    sac_ia.setMaxCorrespondenceDistance(0.05);  // Max distance between correspondences
	    sac_ia.setMaximumIterations(1000);  // Max iterations to run RANSAC
	    sac_ia.setMinSampleDistance(0.01);  // Minimum sample distance
	    sac_ia.setNumberOfSamples(3);  // Minimum number of samples to define a transformation

	    // Apply the RANSAC-based alignment
	    sac_ia.align(*aligned_object);

	    // End timing the RANSAC alignment
	    end_time = std::chrono::high_resolution_clock::now();
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "RANSAC alignment took %ld ms", duration.count());

	    if (sac_ia.hasConverged())
	    {
		RCLCPP_INFO(this->get_logger(), "SAC-IA has converged, transformation found.");

		// Transform the object point cloud (custom pod) to the scene
		pcl::transformPointCloud(*custom_object_, *aligned_object, sac_ia.getFinalTransformation());

		// Publish the aligned object cloud
		sensor_msgs::msg::PointCloud2 output_aligned;
		pcl::toROSMsg(*aligned_object, output_aligned);
		output_aligned.header.stamp = this->get_clock()->now();
		output_aligned.header.frame_id = "UR5_camera_depth_optical_frame";  // Ensure this matches your RViz fixed frame
		point_cloud_publisher_->publish(output_aligned);
	    }
	    else
	    {
		RCLCPP_WARN(this->get_logger(), "SAC-IA did not converge.");
	    }

	    // Publish the filtered point cloud just before matching
	    sensor_msgs::msg::PointCloud2 output_pre_segmentation;
	    pcl::toROSMsg(*cloud_filtered_xyz, output_pre_segmentation);
	    output_pre_segmentation.header.stamp = this->get_clock()->now();
	    output_pre_segmentation.header.frame_id = "UR5_camera_depth_optical_frame"; // Ensure this matches your RViz fixed frame
	    pre_segmentation_publisher_->publish(output_pre_segmentation);
	}

    void filter_by_rgb(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud)
    {
        // Define the target RGB color and the tolerance range (±15)
        const int r_min = 120 - 30;
        const int r_max = 120 + 30;
        const int g_min = 59 - 30;
        const int g_max = 59 + 30;
        const int b_min = 37 - 30;
        const int b_max = 37 + 30;

        // Filter the points based on RGB values
        for (const auto& point : input_cloud->points)
        {
            int r = static_cast<int>(point.r);
            int g = static_cast<int>(point.g);
            int b = static_cast<int>(point.b);

            // Check if the point's RGB value is within the target range
            if (r >= r_min && r <= r_max &&
                g >= g_min && g <= g_max &&
                b >= b_min && b <= b_max)
            {
                output_cloud->points.push_back(point);
            }
        }
    }

    void publish_custom_object_to_world()
    {
        // Publish custom object at world frame (center of world)
        sensor_msgs::msg::PointCloud2 output_custom_object;
        pcl::toROSMsg(*custom_object_, output_custom_object);
        output_custom_object.header.stamp = this->get_clock()->now();
        output_custom_object.header.frame_id = "UR5_camera_link";  // Center of the world frame
        custom_object_publisher_->publish(output_custom_object);

        RCLCPP_INFO(this->get_logger(), "Publishing custom object at world frame.");
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr custom_object_;  // Store the loaded custom object

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create and spin the node
    rclcpp::spin(std::make_shared<PointCloudCustomObjectMatcher>());

    rclcpp::shutdown();
    return 0;
}

