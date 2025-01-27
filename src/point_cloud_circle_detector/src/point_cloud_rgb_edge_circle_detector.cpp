#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h> // Correct header for ROS 2 conversion
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
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
    }

private:
    void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {

        RCLCPP_INFO(this->get_logger(), "PointCloud2 message received");
	    // Declare time variables for timing different steps
	    auto start_time = std::chrono::high_resolution_clock::now();
	    auto end_time = std::chrono::high_resolution_clock::now();
	    std::chrono::duration<double> duration;
        // Create a PointCloud object to hold the point cloud data

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Convert the ROS PointCloud2 message to a PCL PointCloud
        pcl::fromROSMsg(*msg, *cloud_rgb); // Conversion function provided by pcl_conversions

        // Check if the cloud is organized
        if (cloud_rgb->isOrganized())
        {
            std::cout << "Cloud cloud_rgb is organized" << std::endl;
        }
        else
        {
            std::cout << "Cloud cloud_rgb is not organized" << std::endl;
        }

        // Define the color range to filter
        uint8_t min_red = 80, max_red = 155;
        uint8_t min_green = 20, max_green = 100;
        uint8_t min_blue = 0, max_blue = 70;

        // Create conditions for filtering based on RGB values
        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr red_condition(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT, min_red));
        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr red_condition_max(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::LT, max_red));

        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr green_condition(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GT, min_green));
        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr green_condition_max(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::LT, max_green));

        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr blue_condition(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::GT, min_blue));
        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr blue_condition_max(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::LT, max_blue));

        // Combine the conditions into one
        pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
        color_cond->addComparison(red_condition);
        color_cond->addComparison(red_condition_max);
        color_cond->addComparison(green_condition);
        color_cond->addComparison(green_condition_max);
        color_cond->addComparison(blue_condition);
        color_cond->addComparison(blue_condition_max);

        // Apply the filter
        pcl::ConditionalRemoval<pcl::PointXYZRGB> cond_removal;
        cond_removal.setCondition(color_cond);
        cond_removal.setInputCloud(cloud_rgb);
        cond_removal.setKeepOrganized(true);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        cond_removal.filter(*cloud_rgb_filtered);

        // Check if the cloud is organized
        if (cloud_rgb_filtered->isOrganized())
        {
            std::cout << "Cloud cloud_rgb_filtered is organized" << std::endl;
        }
        else
        {
            std::cout << "Cloud cloud_rgb_filtered is not organized" << std::endl;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        cloud->width = cloud_rgb_filtered->width;
        cloud->height = cloud_rgb_filtered->height;
        cloud->is_dense = false;
        cloud->points.resize(cloud_rgb_filtered->width * cloud_rgb_filtered->height);

        for (size_t i = 0; i < cloud_rgb_filtered->size(); ++i)
        {
            cloud->points[i].x = cloud_rgb_filtered->points[i].x;
            cloud->points[i].y = cloud_rgb_filtered->points[i].y;
            cloud->points[i].z = cloud_rgb_filtered->points[i].z;
        }

        // Check if the cloud is organized
        if (cloud->isOrganized())
        {
            std::cout << "Cloud is organized" << std::endl;
        }
        else
        {
            std::cout << "Cloud is not organized" << std::endl;
        }

        // Output the dimensions of the cloud
        std::cout << "PointCloud Dimensions: width = " << cloud->width
                  << ", height = " << cloud->height << std::endl;

        // Set up the edge detection object using OrganizedEdgeBase
        pcl::OrganizedEdgeBase<pcl::PointXYZ, pcl::Label> edge_detector;
        edge_detector.setInputCloud(cloud);

        // Prepare output containers for edge labels and indices
        pcl::PointCloud<pcl::Label> labels;           // Store the labels (e.g., edge, non-edge, boundary)
        std::vector<pcl::PointIndices> label_indices; // Store indices of points corresponding to each edge type

        // Perform the edge detection
        edge_detector.compute(labels, label_indices);

        // Create containers for points classified as edges, boundaries, or non-boundaries
        pcl::PointCloud<pcl::PointXYZ>::Ptr edges(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr non_boundary(new pcl::PointCloud<pcl::PointXYZ>);

        edges->width = cloud->width;
        edges->height = cloud->height;
        edges->is_dense = false;
        edges->points.resize(cloud->width * cloud->height);

        boundary->width = cloud->width;
        boundary->height = cloud->height;
        boundary->is_dense = false;
        boundary->points.resize(cloud->width * cloud->height);

        non_boundary->width = cloud->width;
        non_boundary->height = cloud->height;
        non_boundary->is_dense = false;
        non_boundary->points.resize(cloud->width * cloud->height);

        pcl::PointCloud<pcl::PointXYZ>::Ptr unorganised_edges(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr unorganised_non_boundary(new pcl::PointCloud<pcl::PointXYZ>);


        // Iterate over the labels and classify points into edge, boundary, and non-boundary
        for (size_t i = 0; i < labels.size(); ++i)
        {
            int label = labels[i].label; // Get the label of the current point

            // Check if the point is classified as an edge (high curvature)
            if (label & pcl::OrganizedEdgeBase<pcl::PointXYZ, pcl::Label>::EDGELABEL_OCCLUDING)
            {
                edges->points[i].x = cloud->at(i).x;
                edges->points[i].y = cloud->at(i).y;
                edges->points[i].z = cloud->at(i).z;
                unorganised_edges->points.push_back(cloud->at(i));
            }
            // Check if the point is classified as boundary (occluded)
            else if (label & pcl::OrganizedEdgeBase<pcl::PointXYZ, pcl::Label>::EDGELABEL_OCCLUDED)
            {
                boundary->points[i].x = cloud->at(i).x;
                boundary->points[i].y = cloud->at(i).y;
                boundary->points[i].z = cloud->at(i).z;
                // std::cout << cloud->points[i] << std::endl;
            }
            // Otherwise, classify as non-boundary
            else
            {
                non_boundary->points[i].x = cloud->at(i).x;
                non_boundary->points[i].y = cloud->at(i).y;
                non_boundary->points[i].z = cloud->at(i).z;
                unorganised_non_boundary->points.push_back(cloud->at(i));
            }
        }
        std::cout << "Number of Labels as edges: " << labels.size() << std::endl;
        // Output the number of points in each category
        std::cout << "Number of points classified as edges: " << edges->points.size() << std::endl;
        std::cout << "Number of points classified as boundaries: " << boundary->points.size() << std::endl;
        std::cout << "Number of points classified as non-boundaries: " << non_boundary->points.size() << std::endl;


        std::cout << "Number of points classified as unorganised_edges: " << unorganised_edges->points.size() << std::endl;
        std::cout << "Number of points classified as unorganised_non_boundary: " << unorganised_non_boundary->points.size() << std::endl;

        // Publish the filtered point cloud just before matching
	    sensor_msgs::msg::PointCloud2 output_pre_segmentation;
	    pcl::toROSMsg(*unorganised_non_boundary, output_pre_segmentation);
	    output_pre_segmentation.header.stamp = this->get_clock()->now();
	    output_pre_segmentation.header.frame_id = "UR5_camera_depth_optical_frame"; // Ensure this matches your RViz fixed frame
	    pre_segmentation_publisher_->publish(output_pre_segmentation);

	    pcl::SACSegmentation<pcl::PointXYZ> seg;
	    seg.setInputCloud(cloud_filtered);
	    seg.setMethodType(pcl::SAC_RANSAC);
	    seg.setModelType(pcl::SACMODEL_CYLINDER); // Detect cylinders (or circles)
	    seg.setMaxIterations(1000);
	    seg.setDistanceThreshold(0.01)
	    seg.setRadiusLimits(0.0, 0.15);  // Radius in meters, 0.15 meters = 15 cm

	    // Prepare the output
	    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	    // Call segment function to obtain set of inlier indices and model coefficients
	    seg.segment(*inliers, *coefficients);

	    if (sac_ia.hasConverged())
	    {
		RCLCPP_INFO(this->get_logger(), "SAC-IA has converged, transformation found.");

	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>);
	    for (int index : inliers->indices)
	    {
		cloud_inliers->points.push_back(cloud_filtered->points[index]);
	    }

		// Publish the aligned object cloud
		sensor_msgs::msg::PointCloud2 output_aligned;
		pcl::toROSMsg(*aligned_object, output_aligned);
		output_aligned.header.stamp = this->get_clock()->now();
		output_aligned.header.frame_id = "UR5_camera_depth_optical_frame";  // Ensure this matches your camera frame
		point_cloud_publisher_->publish(output_aligned);
	    }
	    else
	    {
		RCLCPP_WARN(this->get_logger(), "SAC-IA alignment did not converge.");
	    }
	}

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr custom_object_;
};

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the subscriber node
    rclcpp::spin(std::make_shared<PointCloudCustomObjectMatcher>());


    // Shutdown ROS 2 when done
    rclcpp::shutdown();

    return 0;
}
