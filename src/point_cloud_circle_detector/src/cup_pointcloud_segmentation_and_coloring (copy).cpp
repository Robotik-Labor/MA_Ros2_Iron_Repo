#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudGenerator : public rclcpp::Node {
public:
    PointCloudGenerator() : Node("pointcloud_generator") {
        // Subscriber für Depth, RGB und CameraInfo Topics
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/UR5/UR5_camera/aligned_depth_to_color/image_raw", 10, 
            std::bind(&PointCloudGenerator::pointCloudCallback, this, std::placeholders::_1)
        );

        rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/UR5/UR5_camera/color/image_raw", 10,
            std::bind(&PointCloudGenerator::rgbCallback, this, std::placeholders::_1)
        );

        depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/UR5/UR5_camera/aligned_depth_to_color/camera_info", 10,
            std::bind(&PointCloudGenerator::depthInfoCallback, this, std::placeholders::_1)
        );

        pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/generated_pointcloud", 10
        );
    }

private:
    void depthInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        camera_info_ = *msg;
    }

    void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        current_rgb_ = msg;
    }

    void pointCloudCallback(const sensor_msgs::msg::Image::SharedPtr depth_msg) {
        if (!current_rgb_ || camera_info_.height == 0) return;

        // Convert depth image to OpenCV
        cv_bridge::CvImagePtr depth_cv_ptr;
        cv_bridge::CvImagePtr rgb_cv_ptr;
        
        try {
            depth_cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
            rgb_cv_ptr = cv_bridge::toCvCopy(current_rgb_, sensor_msgs::image_encodings::RGB8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "CV Bridge exception: %s", e.what());
            return;
        }

        // Erstelle Point Cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        cloud->width = depth_msg->width;
        cloud->height = depth_msg->height;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);

        for (int v = 0; v < depth_msg->height; v++) {
            for (int u = 0; u < depth_msg->width; u++) {
                // Tiefenwert extrahieren
                uint16_t depth = depth_cv_ptr->image.at<uint16_t>(v, u);
                
                // Nur gültige Tiefen verarbeiten
                if (depth == 0) continue;

                // Konvertiere Pixel zu 3D-Koordinaten
                float z = depth * 0.001; // Umrechnung in Meter
                float x = (u - camera_info_.k[2]) * z / camera_info_.k[0];
                float y = (v - camera_info_.k[5]) * z / camera_info_.k[4];

                // RGB-Wert extrahieren
                cv::Vec3b rgb = rgb_cv_ptr->image.at<cv::Vec3b>(v, u);

                // Punkt zur Point Cloud hinzufügen
                pcl::PointXYZRGB point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.r = rgb[0];
                point.g = rgb[1];
                point.b = rgb[2];

                cloud->points[v * depth_msg->width + u] = point;
            }
        }

        // Publish Point Cloud
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header = depth_msg->header;
        pointcloud_pub_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

    sensor_msgs::msg::Image::SharedPtr current_rgb_;
    sensor_msgs::msg::CameraInfo camera_info_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
