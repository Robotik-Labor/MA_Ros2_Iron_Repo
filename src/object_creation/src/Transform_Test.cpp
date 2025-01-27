#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TfListener : public rclcpp::Node
{
public:
    TfListener()
    : Node("tf_listener")
    {
        // Initialize tf2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Call the function to get the transformation
        getTransformation();
    }

private:
    void getTransformation()
    {
        try
        {
            // Wait for the transform from 'world' to 'camera_link'
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "UR5_base_link",  // target frame
                "UR5_camera_link",  // source frame
                rclcpp::Time(0),  // get the latest available transform
                std::chrono::seconds(1));  // timeout

            RCLCPP_INFO(this->get_logger(), "Transform from UR5_base_link to camera_link: \n"
                "Translation: x = %f, y = %f, z = %f \n"
                "Rotation: x = %f, y = %f, z = %f, w = %f",
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfListener>());
    rclcpp::shutdown();
    return 0;
}

