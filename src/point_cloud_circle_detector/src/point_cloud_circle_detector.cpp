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

// ROS2 Publisher for PointCloud2
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pre_segmentation_publisher_;  // New publisher for pre-segmentation point cloud

class PointCloudCircleDetector : public rclcpp::Node
{
public:
    PointCloudCircleDetector()
        : Node("point_cloud_circle_detector")
    {
        // Publisher to publish PointCloud2 data with Reliable QoS
        point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/UR5/detected_circle_point_cloud", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

        // Publisher for the pre-segmentation (filtered) point cloud
        pre_segmentation_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/UR5/pre_segmentation_point_cloud", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

        // Create subscription to PointCloud2
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/UR5/UR5_camera/depth/color/points", 1,
            std::bind(&PointCloudCircleDetector::listener_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PointCloud Circle Detector Node has been started.");
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

        // Use sample consensus (RANSAC) to detect 3D circles
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CIRCLE3D);  // 3D circle model
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations (10000);
        seg.setDistanceThreshold(0.01);  // Max distance to model for inliers
        seg.setRadiusLimits (0, 0.08);
        Eigen::Vector3f axis(0.0f, 0.0f, 1.0f);  // Camera Z-axis
        seg.setAxis(axis);  // Restrict cylinder detection along the Z-axis

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remaining(cloud_filtered);

        // Create a single PointCloud to store all detected circles
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_circles(new pcl::PointCloud<pcl::PointXYZRGB>());

        // Detect multiple circles
        while (cloud_remaining->points.size() > 100)  // Stop when remaining points are too few
        {
            seg.setInputCloud(cloud_remaining);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No circle found in the remaining cloud.");
                break;
            }

            // Extract the inliers corresponding to the detected circle
            pcl::PointCloud<pcl::PointXYZ>::Ptr circle_points(new pcl::PointCloud<pcl::PointXYZ>);
            for (int index : inliers->indices)
            {
                circle_points->points.push_back(cloud_remaining->points[index]);
            }

            // Get the radius of the detected circle
            float detected_radius = coefficients->values[3];  // The radius is the 4th value in ModelCoefficients

            // Always remove the detected points (inliers), regardless of the radius
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_remaining);
            extract.setIndices(inliers);
            extract.setNegative(true);  // Remove the inliers (i.e., the circle points)
            extract.filter(*cloud_remaining);  // Update the remaining point cloud

            // Only process circles with radius in the desired range
            if (detected_radius >= 0.035 && detected_radius <= 0.045)  // Example range for 8 cm diameter (4 cm radius)
            {
                // Store detected circle
                RCLCPP_INFO(this->get_logger(), "Circle detected with radius %.3f meters", detected_radius);

                // Assign a unique color to each circle
                int r = rand() % 256;  // Random red component
                int g = rand() % 256;  // Random green component
                int b = rand() % 256;  // Random blue component

                // Loop through the points in the current circle and assign colors
                for (const auto& point : circle_points->points)
                {
                    pcl::PointXYZRGB colored_point;
                    colored_point.x = point.x;
                    colored_point.y = point.y;
                    colored_point.z = point.z;
                    colored_point.r = r;
                    colored_point.g = g;
                    colored_point.b = b;

                    // Add colored point to the final point cloud
                    all_circles->points.push_back(colored_point);
                }
            }
            else
            {
                // If the circle does not match the radius condition, simply log and skip adding it
                RCLCPP_INFO(this->get_logger(), "Circle radius %.3f m is outside the desired range.", detected_radius);
            }
        }

        // Convert the final point cloud (with multiple colored circles) to a PointCloud2 message
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*all_circles, output);
        output.header.stamp = this->get_clock()->now();
        output.header.frame_id = "UR5_camera_depth_optical_frame"; // Ensure this matches your RViz fixed frame

        // Publish the point cloud containing all detected circles
        RCLCPP_INFO(this->get_logger(), "Publishing PointCloud2 message with %zu points", all_circles->points.size());
        point_cloud_publisher_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create and spin the node
    rclcpp::spin(std::make_shared<PointCloudCircleDetector>());

    rclcpp::shutdown();
    return 0;
}

