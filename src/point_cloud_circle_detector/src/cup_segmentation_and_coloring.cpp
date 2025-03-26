#define PCL_NO_PRECOMPILE
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>

class CupPointCloudProcessor : public rclcpp::Node 
{
public:
    CupPointCloudProcessor() : Node("cup_pointcloud_processor")
    {
        // Declare parameters
        this->declare_parameter<std::string>("input_pointcloud_topic", "/UR5/UR5_camera/depth/color/points");
        this->declare_parameter<std::string>("cup_pixels_topic", "/UR5/cup_pixel_coordinates");
        this->declare_parameter<std::string>("output_pointcloud_topic", "/UR5/cups_pointcloud");

        // Get parameters
        std::string input_pointcloud_topic = this->get_parameter("input_pointcloud_topic").as_string();
        std::string cup_pixels_topic = this->get_parameter("cup_pixels_topic").as_string();
        std::string output_pointcloud_topic = this->get_parameter("output_pointcloud_topic").as_string();

        // Subscribers
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_pointcloud_topic, 10, 
            std::bind(&CupPointCloudProcessor::pointcloud_callback, this, std::placeholders::_1)
        );

        cup_pixels_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            cup_pixels_topic, 10,
            std::bind(&CupPointCloudProcessor::cup_pixels_callback, this, std::placeholders::_1)
        );

        // Publishers
        cups_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_pointcloud_topic, 10
        );
    }

private:
    // ROS2 Subscribers and Publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cup_pixels_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cups_pointcloud_pub_;

    // Store latest point cloud and cup pixels
    sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
    std::vector<std::vector<std::pair<int, int>>> cup_pixel_groups_;

    void cup_pixels_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Clear previous cup pixel groups
        cup_pixel_groups_.clear();

        // Log message details
        RCLCPP_INFO(this->get_logger(), "Received Pixel Coordinates Message");
        RCLCPP_INFO(this->get_logger(), "Message Data Size: %zu", msg->data.size());
        RCLCPP_INFO(this->get_logger(), "Layout Dimensions: %zu", msg->layout.dim.size());

        // Ensure we have at least two dimensions
        if (msg->layout.dim.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Invalid pixel coordinates message");
            return;
        }

        int num_pixels = msg->layout.dim[0].size;
        RCLCPP_INFO(this->get_logger(), "Number of Pixels: %d", num_pixels);
        
        // Reconstruct pixel groups
        std::vector<std::pair<int, int>> current_group;
        for (int i = 0; i < num_pixels; i++) {
            int x = msg->data[i * 2];
            int y = msg->data[i * 2 + 1];
            current_group.emplace_back(x, y);
            
            // Log first few pixels
            if (i < 5) {
                RCLCPP_INFO(this->get_logger(), "Pixel %d: (%d, %d)", i, x, y);
            }
        }
        
        cup_pixel_groups_.push_back(current_group);

        // Additional logging about point cloud
        if (latest_pointcloud_) {
            RCLCPP_INFO(this->get_logger(), "Point Cloud Width: %d", latest_pointcloud_->width);
            RCLCPP_INFO(this->get_logger(), "Point Cloud Height: %d", latest_pointcloud_->height);
            RCLCPP_INFO(this->get_logger(), "Point Cloud Point Step: %d", latest_pointcloud_->point_step);
        }

        // If we have both point cloud and cup pixels, process
        if (latest_pointcloud_) {
            process_cups_pointcloud();
        }
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Store the latest point cloud
        latest_pointcloud_ = msg;

        // If we have both point cloud and cup pixels, process
        if (!cup_pixel_groups_.empty()) {
            process_cups_pointcloud();
        }
    }

    void process_cups_pointcloud()
    {
        // Existing code, but with more logging
        RCLCPP_INFO(this->get_logger(), "Starting Point Cloud Processing");
        
        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
            input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*latest_pointcloud_, *input_cloud);

        RCLCPP_INFO(this->get_logger(), "Input Cloud Size: %zu", input_cloud->size());

        // Output cloud for all cups
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
            output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Process each cup group
        for (size_t cup_index = 0; cup_index < cup_pixel_groups_.size(); ++cup_index)
        {
            // Generate a unique color for this cup
            uint8_t r = (cup_index * 50) % 256;
            uint8_t g = (cup_index * 100) % 256;
            uint8_t b = (cup_index * 150) % 256;
            uint32_t rgb_color = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

            // Create indices for this cup
            pcl::PointIndices::Ptr cup_indices(new pcl::PointIndices());

            // Find points within the cup pixel coordinates
            for (const auto& pixel : cup_pixel_groups_[cup_index])
            {
                int x = pixel.first;
                int y = pixel.second;

                // Calculate linear index in the point cloud
                size_t index = y * latest_pointcloud_->width + x;

                // Ensure index is valid
                if (index < input_cloud->size()) {
                    cup_indices->indices.push_back(index);
                }
            }

            // Extract points for this cup
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
                cup_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(input_cloud);
            extract.setIndices(cup_indices);
            extract.filter(*cup_cloud);

            // Color the cup points
            for (auto& point : cup_cloud->points) {
                point.rgb = *reinterpret_cast<float*>(&rgb_color);
            }

            // Concatenate to output cloud
            *output_cloud += *cup_cloud;
        }

        // Convert back to ROS message
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*output_cloud, output_msg);
        
        // Use the same header as input point cloud
        output_msg.header = latest_pointcloud_->header;

        // Publish the result
        cups_pointcloud_pub_->publish(output_msg);

        // Log processing details
        RCLCPP_INFO(this->get_logger(), 
            "Processed %zu cup groups, total points: %zu", 
            cup_pixel_groups_.size(), 
            output_cloud->size()
        );
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CupPointCloudProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
