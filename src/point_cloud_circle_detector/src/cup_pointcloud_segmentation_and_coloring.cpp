#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <random>
#include <tuple>

class PointCloudGenerator : public rclcpp::Node {
public:
    PointCloudGenerator() : Node("pointcloud_generator") {
        // Subscriber für Depth, RGB und CameraInfo Topics
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/UR5/UR5_camera/aligned_depth_to_color/image_raw", 1, 
            std::bind(&PointCloudGenerator::depthCallback, this, std::placeholders::_1)
        );

        rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/UR5/UR5_camera/color/image_raw", 1,
            std::bind(&PointCloudGenerator::rgbCallback, this, std::placeholders::_1)
        );

        depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/UR5/UR5_camera/aligned_depth_to_color/camera_info", 1,
            std::bind(&PointCloudGenerator::depthInfoCallback, this, std::placeholders::_1)
        );

        cup_information_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/UR5/cup_pixel_coordinates", 1, std::bind(&PointCloudGenerator::pointCloudCallback, this, std::placeholders::_1));
        

        pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/UR5/plantpod_reduced_pointcloud", 10
        );
        
    }

private:

    bool rotate_by_180 = true;
    


    std::tuple<int, int, int> get_random_rgb_from_int(int input) {
        // Seed the random number generator with the input integer
        std::mt19937 rng(input);  // Mersenne Twister RNG
        std::uniform_int_distribution<int> dist(0, 255);  // Uniform distribution for RGB values (0-255)

        // Generate the RGB values
        int red = dist(rng);    // Generate random value for Red channel
        int green = dist(rng);  // Generate random value for Green channel
        int blue = dist(rng);   // Generate random value for Blue channel

        // Return the RGB values as a tuple
        return std::make_tuple(red, green, blue);
    }

    void depthInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        camera_info_ = *msg;
    }

    void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        current_rgb_ = msg;
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        current_depth_ = msg;
    }

    void pointCloudCallback(const std_msgs::msg::Float32MultiArray::SharedPtr cup_msg) {
        if (!current_rgb_ || !current_depth_) return;
        RCLCPP_INFO(get_logger(), "TEEEEEEEEEEEESt");
        // Convert depth image to OpenCV
        cv_bridge::CvImagePtr depth_cv_ptr;
        cv_bridge::CvImagePtr rgb_cv_ptr;

        RCLCPP_INFO(get_logger(), "Depth image encoding: %s", current_depth_->encoding.c_str());
        RCLCPP_INFO(get_logger(), "RGB image encoding: %s", current_rgb_->encoding.c_str());
        
        try {
            depth_cv_ptr = cv_bridge::toCvCopy(current_depth_, sensor_msgs::image_encodings::TYPE_16UC1);
            rgb_cv_ptr = cv_bridge::toCvCopy(current_rgb_, sensor_msgs::image_encodings::RGB8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "CV Bridge exception: %s", e.what());
            return;
        }

        const std::vector<float> &data = cup_msg->data;

        // Log the data to inspect the contents
        RCLCPP_INFO(get_logger(), "Received data: ");
        //for (size_t i = 0; i < data.size(); ++i) {
        //    RCLCPP_INFO(get_logger(), "Data[%zu]: %f", i, data[i]);
        //}

        // Erstelle Point Cloud
        RCLCPP_INFO(get_logger(), "Width: %u", current_depth_->width);
        RCLCPP_INFO(get_logger(), "Height: %u", current_depth_->height);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        cloud->width = current_depth_->width;
        cloud->height = current_depth_->height;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);
        cv::Mat rotated_depth_image;
        cv::Mat rotated_rgb_image;
        cv::Mat hsv_image;
        if (rotate_by_180) {
            cv::rotate(depth_cv_ptr->image, rotated_depth_image, cv::ROTATE_180);
            RCLCPP_INFO(get_logger(), "Rotated Width: %u", rotated_depth_image.cols);
            RCLCPP_INFO(get_logger(), "Rotated Height: %u", rotated_depth_image.rows);
            cv::rotate(rgb_cv_ptr->image, rotated_rgb_image, cv::ROTATE_180);
    	    RCLCPP_INFO(get_logger(), "Rotated RGB Width: %u", rotated_rgb_image.cols);
            RCLCPP_INFO(get_logger(), "Rotated RGB Height: %u", rotated_rgb_image.rows);
            cv::cvtColor(rotated_rgb_image, hsv_image, cv::COLOR_RGB2HSV);
        }
        else{
	    cv::cvtColor(rgb_cv_ptr->image, hsv_image, cv::COLOR_RGB2HSV);
	}
	// Define the red color range
	cv::Mat mask1, mask2;
	cv::inRange(hsv_image, cv::Scalar(0, 70, 50), cv::Scalar(20, 255, 255), mask1);
	cv::inRange(hsv_image, cv::Scalar(160, 70, 50), cv::Scalar(180, 255, 255), mask2);

	// Combine the masks
	cv::Mat mask = mask1 | mask2;
	
        for (size_t i = 0; i < data.size(); i += 3) //
        {       
                uint16_t depth;
                cv::Vec3b rgb;
                // Tiefenwert extrahieren
                if (rotate_by_180) {
                    depth = rotated_depth_image.at<uint16_t>(data[i+2], data[i+1]);
                    //hsv = hsv_image.at<cv::Vec3b>(data[i+2], data[i+1]);
                }else{
                    depth = depth_cv_ptr->image.at<uint16_t>(data[i+2], data[i+1]);
                    //rgb = hsv_image.at<cv::Vec3b>(data[i+2], data[i+1]);
                }
                

                // Nur gültige Tiefen verarbeiten
                if (depth == 0 || depth <= 500 || mask.at<uchar>(data[i+2], data[i+1]) == 0) {
		    continue;
		}

                float z = depth * 0.001; // Umrechnung in Meter

                float x, y;
                if (rotate_by_180) {
                    x = ((current_depth_->width - camera_info_.k[2]) - data[i+1] ) * z / camera_info_.k[0];
                    y = ((current_depth_->height - camera_info_.k[5])  - data[i+2] ) * z / camera_info_.k[4];
                    //RCLCPP_INFO(get_logger(), "Center X: %f", camera_info_.k[2]);
                    //RCLCPP_INFO(get_logger(), "Center Y: %f", camera_info_.k[5]);

                } else {
                    // Normal conversion
                    x = (data[i+1] - camera_info_.k[2]) * z / camera_info_.k[0];
                    y = (data[i+2] - camera_info_.k[5]) * z / camera_info_.k[4];
                }

                // RGB-Wert extrahieren
                //cv::Vec3b rgb = rgb_cv_ptr->image.at<cv::Vec3b>(v, u);

                // Punkt zur Point Cloud hinzufügen
                pcl::PointXYZRGB point;
                point.x = x;
                point.y = y;
                point.z = z;
                auto [red, green, blue] = get_random_rgb_from_int(data[i]);
                point.r = red;
                point.g = green;
                point.b = blue;

                cloud->points[data[i+2] * current_depth_->width + data[i+1]] = point;
                //RCLCPP_INFO(get_logger(), "Publishinnpoints");
            
        }



        // Publish Point Cloud
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header = current_depth_->header;
        pointcloud_pub_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cup_information_;


    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

    sensor_msgs::msg::Image::SharedPtr current_rgb_;
    sensor_msgs::msg::Image::SharedPtr current_depth_;
    sensor_msgs::msg::CameraInfo camera_info_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
