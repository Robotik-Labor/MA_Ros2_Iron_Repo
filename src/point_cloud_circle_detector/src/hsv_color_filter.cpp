// color_filter_node.cpp - ROS2 node for HSV-based green filtering of point clouds
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>

class ColorFilterNode : public rclcpp::Node {
public:
  ColorFilterNode() : Node("color_filter_node") {
    // Declare parameters with default values for green detection
    this->declare_parameter("h_min", 80.0);  // Hue min for green
    this->declare_parameter("h_max", 150.0);  // Hue max for green
    this->declare_parameter("s_min", 0.3);  // Saturation min
    this->declare_parameter("s_max", 1.0); // Saturation max
    this->declare_parameter("v_min", 0.3);  // Value min
    this->declare_parameter("v_max", 1.0); // Value max
    
    // Create publisher and subscriber
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/UR5/UR5_camera/depth/color/points/green_filtered", 10);
    
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/UR5/UR5_camera/depth/color/points", 10,
      std::bind(&ColorFilterNode::cloud_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Color filter node initialized with HSV filtering");
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  
  // Helper function to convert RGB to HSV
 void hsv2rgb(double h, double s, double v, double &r, double &g, double &b) {
    double hh, p, q, t, ff;
    long i;

    if(s <= 0.0) {  // < is bogus, just shuts up warnings
        r = v;
        g = v;
        b = v;
        return;
    }

    hh = h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = v * (1.0 - s);
    q = v * (1.0 - (s * ff));
    t = v * (1.0 - (s * (1.0 - ff)));

    switch(i) {
        case 0:
            r = v;
            g = t;
            b = p;
            break;
        case 1:
            r = q;
            g = v;
            b = p;
            break;
        case 2:
            r = p;
            g = v;
            b = t;
            break;
        case 3:
            r = p;
            g = q;
            b = v;
            break;
        case 4:
            r = t;
            g = p;
            b = v;
            break;
        case 5:
        default:
            r = v;
            g = p;
            b = q;
            break;
    }
}



  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_msg) {
    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*input_msg, *cloud);
    
    // Get HSV parameters
    double h_min = this->get_parameter("h_min").as_double();
    double h_max = this->get_parameter("h_max").as_double();
    double s_min = this->get_parameter("s_min").as_double();
    double s_max = this->get_parameter("s_max").as_double();
    double v_min = this->get_parameter("v_min").as_double();
    double v_max = this->get_parameter("v_max").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Input HSV values - Min Hue: %f, Max Hue: %f, Min Saturation: %f, Max Saturation: %f, Min Value: %f, Max Value: %f", 
            h_min, h_max, s_min, s_max, v_min, v_max);
    
    // Create a new point cloud for the filtered result
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    filtered_cloud->reserve(cloud->points.size());
    double min_red ,max_red, min_green, max_green, min_blue, max_blue;
    hsv2rgb(h_max,s_max,v_max,max_red,max_green,max_blue);
    hsv2rgb(h_min,s_min,v_min,min_red,min_green,min_blue);
    //min_red = min_red * 255.0;
    //max_red = max_red * 255.0;
    //min_green = min_green * 255.0;
    //max_green = max_green * 255.0;
    //min_blue = min_blue * 255.0;
    //max_blue = max_blue * 255.0;
    int min_red_int = static_cast<int>(min_red * 255.0);  
    int max_red_int = static_cast<int>(max_red * 255.0);  
    int min_green_int = static_cast<int>(min_green * 255.0);  
    int max_green_int = static_cast<int>(max_green * 255.0);  
    int min_blue_int = static_cast<int>(min_blue * 255.0);  
    int max_blue_int = static_cast<int>(max_blue * 255.0);  
    
    if (min_red_int > max_red_int) {
    std::swap(min_red_int, max_red_int);  // Swap if min is larger than max
}

// Conditional check and swap min/max for green
if (min_green_int > max_green_int) {
    std::swap(min_green_int, max_green_int);  // Swap if min is larger than max
}

// Conditional check and swap min/max for blue
if (min_blue_int > max_blue_int) {
    std::swap(min_blue_int, max_blue_int);  // Swap if min is larger than max
}
    
    // Log the converted RGB values
RCLCPP_INFO(this->get_logger(), "Converted RGB values:");
RCLCPP_INFO(this->get_logger(), "Min Red: %d, Max Red: %d", min_red_int, max_red_int);
RCLCPP_INFO(this->get_logger(), "Min Green: %d, Max Green: %d", min_green_int, max_green_int);
RCLCPP_INFO(this->get_logger(), "Min Blue: %d, Max Blue: %d", min_blue_int, max_blue_int);

        // Create conditions for filtering based on RGB values
        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr red_condition(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT, min_red_int));
        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr red_condition_max(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::LT, max_red_int));

        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr green_condition(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GT, min_green_int));
        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr green_condition_max(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::LT, max_green_int));

        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr blue_condition(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::GT, min_blue_int));
        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr blue_condition_max(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::LT, max_blue_int));

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
        cond_removal.setInputCloud(cloud);
        cond_removal.filter(*filtered_cloud);
    
    // Convert filtered cloud back to ROS message
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered_cloud, output_msg);
    output_msg.header = input_msg->header;
    
    // Publish filtered point cloud
    publisher_->publish(output_msg);
    
    RCLCPP_DEBUG(this->get_logger(), 
                "Filtered point cloud: %ld points (from %ld)",
                filtered_cloud->points.size(), cloud->points.size());
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ColorFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
