#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>  // Correct header for ROS 2 conversion
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

class PointCloudSubscriber : public rclcpp::Node {
public:
    PointCloudSubscriber() : Node("point_cloud_subscriber") {
        // Create subscription to PointCloud2
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/UR5/UR5_camera/depth/color/points", 10,
            std::bind(&PointCloudSubscriber::listener_callback, this, std::placeholders::_1)
        );
    }

private:
    void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Create a PointCloud object to hold the point cloud data
        
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Convert the ROS PointCloud2 message to a PCL PointCloud
        pcl::fromROSMsg(*msg, *cloud_rgb);  // Conversion function provided by pcl_conversions
        
        
        	// Check if the cloud is organized
        if (cloud_rgb->isOrganized()) {
            std::cout << "Cloud cloud_rgb is organized" << std::endl;
        } else {
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
        if (cloud_rgb_filtered->isOrganized()) {
            std::cout << "Cloud cloud_rgb_filtered is organized" << std::endl;
        } else {
            std::cout << "Cloud cloud_rgb_filtered is not organized" << std::endl;
        }

	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	cloud ->width = cloud_rgb_filtered->width;
    	cloud ->height = cloud_rgb_filtered->height;
    	cloud ->is_dense = false;
    	cloud->points.resize(cloud_rgb_filtered->width * cloud_rgb_filtered->height);
	
	
	for (size_t i = 0; i < cloud_rgb_filtered->size(); ++i) {
	    cloud->points[i].x = cloud_rgb_filtered->points[i].x;
	    cloud->points[i].y = cloud_rgb_filtered->points[i].y;
	    cloud->points[i].z = cloud_rgb_filtered->points[i].z;
	}


        // Check if the cloud is organized
        if (cloud->isOrganized()) {
            std::cout << "Cloud is organized" << std::endl;
        } else {
            std::cout << "Cloud is not organized" << std::endl;
        }

        // Output the dimensions of the cloud
        std::cout << "PointCloud Dimensions: width = " << cloud->width 
                  << ", height = " << cloud->height << std::endl;

        // Set up the edge detection object using OrganizedEdgeBase
        pcl::OrganizedEdgeBase<pcl::PointXYZ, pcl::Label> edge_detector;
        edge_detector.setInputCloud(cloud);

        // Prepare output containers for edge labels and indices
        pcl::PointCloud<pcl::Label> labels;  // Store the labels (e.g., edge, non-edge, boundary)
        std::vector<pcl::PointIndices> label_indices;  // Store indices of points corresponding to each edge type

        // Perform the edge detection
        edge_detector.compute(labels, label_indices);

        // Create containers for points classified as edges, boundaries, or non-boundaries
        pcl::PointCloud<pcl::PointXYZ>::Ptr edges(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr non_boundary(new pcl::PointCloud<pcl::PointXYZ>);
        
        
        edges ->width = cloud->width;
    	edges ->height = cloud->height;
    	edges ->is_dense = false;
    	edges ->points.resize(cloud->width * cloud->height);        
        
        boundary ->width = cloud->width;
    	boundary ->height = cloud->height;
    	boundary ->is_dense = false;
    	boundary ->points.resize(cloud->width * cloud->height);        

	
	non_boundary ->width = cloud->width;
    	non_boundary ->height = cloud->height;
    	non_boundary ->is_dense = false;
    	non_boundary ->points.resize(cloud->width * cloud->height);        
	
        // Iterate over the labels and classify points into edge, boundary, and non-boundary
        for (size_t i = 0; i < labels.size(); ++i) {
            int label = labels[i].label;  // Get the label of the current point

            // Check if the point is classified as an edge (high curvature)
            if (label & pcl::OrganizedEdgeBase<pcl::PointXYZ, pcl::Label>::EDGELABEL_OCCLUDING) {
                edges ->points[i].x = cloud->at(i).x;
        	edges ->points[i].y = cloud->at(i).y;
        	edges ->points[i].z = cloud->at(i).z;
            }
            // Check if the point is classified as boundary (occluded)
            else if (label & pcl::OrganizedEdgeBase<pcl::PointXYZ, pcl::Label>::EDGELABEL_OCCLUDED) {
        	boundary ->points[i].x = cloud->at(i).x;
        	boundary ->points[i].y = cloud->at(i).y;
        	boundary ->points[i].z = cloud->at(i).z;
                //std::cout << cloud->points[i] << std::endl;
            }
            // Otherwise, classify as non-boundary
            else {
                non_boundary ->points[i].x = cloud->at(i).x;
        	non_boundary ->points[i].y = cloud->at(i).y;
        	non_boundary ->points[i].z = cloud->at(i).z;
            }
        }
	std::cout << "Number of Labels as edges: " << labels.size() << std::endl;
        // Output the number of points in each category
        std::cout << "Number of points classified as edges: " << edges->points.size() << std::endl;
        std::cout << "Number of points classified as boundaries: " << boundary->points.size() << std::endl;
        std::cout << "Number of points classified as non-boundaries: " << non_boundary->points.size() << std::endl;
        
        
        std::cout << "Boundary PointCloud Dimensions: width = " << boundary->width 
                  << ", height = " << boundary->height << std::endl;
                  
        std::cout << boundary << std::endl;
                  

        // Save the edge, boundary, and non-boundary points to PCD files
        std::cout << "Start saving Filtered RGB" << std::endl;
        pcl::io::savePCDFileASCII("/home/buhrmann/Desktop/Pointcloud_Tests/RGB_filterd.pcd", *cloud_rgb_filtered);
        std::cout << "Start saving edges" << std::endl;
        pcl::io::savePCDFileASCII("/home/buhrmann/Desktop/Pointcloud_Tests/output_edges.pcd", *edges);
        std::cout << "Start saving boundary" << std::endl;
        pcl::io::savePCDFileASCII("/home/buhrmann/Desktop/Pointcloud_Tests/output_boundary.pcd", *boundary);
        std::cout << "Start saving non_boundary" << std::endl;
        pcl::io::savePCDFileASCII("/home/buhrmann/Desktop/Pointcloud_Tests/output_non_boundary.pcd", *non_boundary);

        std::cout << "Edge detection completed." << std::endl;
    }
    
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the subscriber node
    auto node = std::make_shared<PointCloudSubscriber>();

    // Spin the node just once to process one message
    rclcpp::spin(node);

    // Shutdown ROS 2 when done
    rclcpp::shutdown();

    return 0;
}

