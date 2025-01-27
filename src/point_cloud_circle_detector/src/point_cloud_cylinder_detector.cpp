#include <pcl/filters/extract_indices.h>  // Include for ExtractIndices
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

// ROS2 Publisher for PointCloud2
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pre_segmentation_publisher_;  // New publisher for pre-segmentation point cloud

class PointCloudCylinderDetector : public rclcpp::Node
{
public:
    PointCloudCylinderDetector()
        : Node("point_cloud_cylinder_detector")
    {
        // Publisher to publish PointCloud2 data with Reliable QoS
        point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/UR5/detected_cylinder_point_cloud", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

        // Publisher for the pre-segmentation (filtered) point cloud
        pre_segmentation_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/UR5/pre_segmentation_point_cloud", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

        // Create subscription to PointCloud2
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/UR5/UR5_camera/depth/color/points", 1,
            std::bind(&PointCloudCylinderDetector::listener_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PointCloud Cylinder Detector Node has been started.");
    }

private:
    void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "PointCloud2 message received");

        // Convert PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Apply radius outlier removal filter
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setRadiusSearch(0.01);  // Radius in meters
        sor.setMinNeighborsInRadius(5);  // Minimum neighbors in radius to be considered a valid point
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        sor.filter(*cloud_filtered);

        // Publish the filtered point cloud just before RANSAC segmentation
        sensor_msgs::msg::PointCloud2 output_pre_segmentation;
        pcl::toROSMsg(*cloud_filtered, output_pre_segmentation);
        output_pre_segmentation.header.stamp = this->get_clock()->now();
        output_pre_segmentation.header.frame_id = "UR5_camera_depth_optical_frame"; // Ensure this matches your RViz fixed frame
        pre_segmentation_publisher_->publish(output_pre_segmentation);

        // Normal estimation
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud_filtered);
        ne.setKSearch(50);
        ne.compute(*cloud_normals);

        // Create segmentation object for cylinder detection
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);  // 3D cylinder model
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.1);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.05);
        seg.setRadiusLimits(0, 0.1);  // Set the radius limits for detection
        seg.setInputCloud(cloud_filtered);
        seg.setInputNormals(cloud_normals);

        // Coefficients and inliers for cylinder
        pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
        seg.segment(*inliers_cylinder, *coefficients_cylinder);

        if (inliers_cylinder->indices.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No cylinder found in the point cloud.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Cylinder detected at position coefficients: %f, %f, %f",
                        coefficients_cylinder->values[0], coefficients_cylinder->values[1],
                        coefficients_cylinder->values[2]);
            RCLCPP_INFO(this->get_logger(), "Cylinder Axis coefficients: %f, %f, %f",
                        coefficients_cylinder->values[3], coefficients_cylinder->values[4],
                        coefficients_cylinder->values[5]);
            RCLCPP_INFO(this->get_logger(), "Cylinder Radius coefficients: %f",
                        coefficients_cylinder->values[6]);

            // Extract the cylinder points
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>);
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers_cylinder);
            extract.setNegative(false);
            extract.filter(*cloud_cylinder);

            // Create a colored point cloud to visualize the detected cylinder
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_cylinders(new pcl::PointCloud<pcl::PointXYZRGB>());

            // Color the cylinder points
            for (const auto &point : cloud_cylinder->points)
            {
                pcl::PointXYZRGB colored_point;
                colored_point.x = point.x;
                colored_point.y = point.y;
                colored_point.z = point.z;
                colored_point.r = 255;  // Red for cylinder points
                colored_point.g = 0;
                colored_point.b = 0;

                // Add colored point to final point cloud
                all_cylinders->points.push_back(colored_point);
            }

            // Convert the final point cloud (with colored cylinders) to a PointCloud2 message
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*all_cylinders, output);
            output.header.stamp = this->get_clock()->now();
            output.header.frame_id = "UR5_camera_depth_optical_frame";

            // Publish the point cloud containing all detected cylinders
            RCLCPP_INFO(this->get_logger(), "Publishing PointCloud2 message with %zu points", all_cylinders->points.size());
            point_cloud_publisher_->publish(output);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create and spin the node
    rclcpp::spin(std::make_shared<PointCloudCylinderDetector>());

    rclcpp::shutdown();
    return 0;
}

