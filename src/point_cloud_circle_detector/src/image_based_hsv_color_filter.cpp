#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class PointCloudGenerator : public rclcpp::Node {
public:
    PointCloudGenerator() : Node("pointcloud_generator") {
        // Subscriber für Depth und RGB Topics
    this->declare_parameter("h_min", 30);  // Hue min for green
    this->declare_parameter("h_max", 90);  // Hue max for green
    this->declare_parameter("s_min", 65);  // Saturation min
    this->declare_parameter("s_max", 210); // Saturation max
    this->declare_parameter("v_min", 40);  // Value min
    this->declare_parameter("v_max", 255); // Value max
        
        
        depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/UR5/UR5_camera/aligned_depth_to_color/camera_info", 1,
            std::bind(&PointCloudGenerator::depthInfoCallback, this, std::placeholders::_1)
        );
   
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/UR5/UR5_camera/aligned_depth_to_color/image_raw", 1, 
            std::bind(&PointCloudGenerator::depthCallback, this, std::placeholders::_1)
        );

        rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/UR5/UR5_camera/color/image_raw", 1,
            std::bind(&PointCloudGenerator::rgbCallback, this, std::placeholders::_1)
        );

        // PointCloud publisher
        pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/UR5/UR5_camera/depth/color/points/green_filtered", 1);
        
        // Image publisher
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("/UR5/image/green_filtered", 1);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    
    cv::Mat current_depth_;
    cv::Mat current_rgb_;

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr depth_msg) {
        // Convert ROS Image message to OpenCV Mat
        try {
            current_depth_ = cv_bridge::toCvCopy(depth_msg, "16UC1")->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "Could not convert depth image: %s", e.what());
        }
    }
    
    void depthInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        camera_info_ = *msg;
    }

    void rgbCallback(const sensor_msgs::msg::Image::SharedPtr rgb_msg) {
        // Convert ROS Image message to OpenCV Mat
        try {
            current_rgb_ = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "Could not convert RGB image: %s", e.what());
        }

        // Apply green color filtering using OpenCV
        cv::Mat hsv_image, green_mask;
        cv::cvtColor(current_rgb_, hsv_image, cv::COLOR_BGR2HSV);
        
        int h_min = this->get_parameter("h_min").as_int();
        int h_max = this->get_parameter("h_max").as_int();
        int s_min = this->get_parameter("s_min").as_int();
        int s_max = this->get_parameter("s_max").as_int();
        int v_min = this->get_parameter("v_min").as_int();
        int v_max = this->get_parameter("v_max").as_int();

        // Define the HSV range for green color detection
        cv::Scalar lower_green(h_min, s_min, v_min); // Lower bound for green
        cv::Scalar upper_green(h_max, s_max, v_max); // Upper bound for green

        // Threshold the HSV image to get only the green areas
        cv::inRange(hsv_image, lower_green, upper_green, green_mask);
        
       

	// Apply the mask to the original RGB image
	cv::Mat filtered_rgb;
	cv::bitwise_and(current_rgb_, current_rgb_, filtered_rgb, green_mask);

	// OPTIONAL: Convert back to RGB if needed (OpenCV loads as BGR by default)
	cv::Mat final_rgb;
	cv::cvtColor(filtered_rgb, final_rgb, cv::COLOR_BGR2RGB);

	// Now publish final_rgb using your ROS publisher
	sensor_msgs::msg::Image::SharedPtr output_msg = cv_bridge::CvImage(
	    std_msgs::msg::Header(), "rgb8", final_rgb).toImageMsg();

	image_pub_->publish(*output_msg);
        
        
        

        // Create a point cloud based on depth and green-filtered RGB
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        cloud->width = current_depth_.cols;
        cloud->height = current_depth_.rows;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);

        for (int v = 0; v < current_depth_.rows; ++v) {
            for (int u = 0; u < current_depth_.cols; ++u) {
                uint16_t depth_value = current_depth_.at<uint16_t>(v, u);
                
                // Skip invalid depth values (0 means no depth data)
                if (depth_value == 0) continue;

                // Convert depth to meters
                float z = depth_value * 0.001f; // Convert depth to meters
                     
                float x = (u - camera_info_.k[2]) * z / camera_info_.k[0];
                float y = (v - camera_info_.k[5]) * z / camera_info_.k[4];

                // Get the RGB value from the original image
                cv::Vec3b rgb = current_rgb_.at<cv::Vec3b>(v, u);

                // Check if the pixel is within the green mask
                if (green_mask.at<uchar>(v, u) != 0) {
                    pcl::PointXYZRGB point;
                    point.x = x;
                    point.y = y;
                    point.z = z;

                    // Set the color of the point based on the filtered green area
                    point.r = rgb[2]; // Red
                    point.g = rgb[1]; // Green
                    point.b = rgb[0]; // Blue

                    // Add the point to the point cloud
                    cloud->points[v * current_depth_.cols + u] = point;
                }
            }
        }
        

        // Publish the filtered point cloud
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header = rgb_msg->header;
        pointcloud_pub_->publish(output);
    }
    
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  sensor_msgs::msg::CameraInfo camera_info_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudGenerator>());
    rclcpp::shutdown();
    return 0;
}

