#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/organized_edge_detection.h>
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
        
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Convert the ROS PointCloud2 message to a PCL PointCloud
        pcl::fromROSMsg(*msg, *cloud);  // Conversion function provided by pcl_conversions

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
	
	std::cout << "Label size: " << labels.size() << std::endl;
        // Output the number of points in each category
        std::cout << "Number of points classified as edges: " << edges->points.size() << std::endl;
        std::cout << "Number of points classified as boundaries: " << boundary->points.size() << std::endl;
        std::cout << "Number of points classified as non-boundaries: " << non_boundary->points.size() << std::endl;
        
        
        std::cout << "Boundary PointCloud Dimensions: width = " << boundary->width 
                  << ", height = " << boundary->height << std::endl;
                  
        std::cout << boundary << std::endl;
                  

        // Save the edge, boundary, and non-boundary points to PCD files
        std::cout << "Start saving edges" << std::endl;
        pcl::io::savePCDFileASCII("/home/buhrmann/Desktop/Pointcloud_Tests/output_edges.pcd", *edges);
        std::cout << "Start saving boundary" << std::endl;
        pcl::io::savePCDFileASCII("/home/buhrmann/Desktop/Pointcloud_Tests/output_boundary.pcd", *boundary);
        std::cout << "Start saving non_boundary" << std::endl;
        pcl::io::savePCDFileASCII("/home/buhrmann/Desktop/Pointcloud_Tests/output_non_boundary.pcd", *non_boundary);

        std::cout << "Edge detection completed." << std::endl;
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

