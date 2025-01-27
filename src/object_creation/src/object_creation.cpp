#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>
#include "shape_msgs/msg/solid_primitive.hpp"
#include <regex>
#include <string>

class ObjectDetectionListener : public rclcpp::Node
{
public:
    ObjectDetectionListener()
        : Node("object_detection_listener")
    {
        // Initialize MoveIt! 2 Planning Scene Interface
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>("UR5");

        // Subscribe to the detected object topic
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "UR5/detected_object", 10, std::bind(&ObjectDetectionListener::detectionCallback, this, std::placeholders::_1));

        // Camera parameters (for RealSense D435i or other camera)
        fx_ = 616.1530;  // Focal length in x direction (in pixels)
        fy_ = 616.1530;  // Focal length in y direction (in pixels)
        cx_ = 320.0;     // Principal point (x)
        cy_ = 240.0;     // Principal point (y)

        RCLCPP_INFO(this->get_logger(), "Ready to listen to detections and place objects in RViz.");
    }

private:
    void detectionCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        try
        {
            std::string detection_data = msg->data;
            RCLCPP_INFO(this->get_logger(), "Received detection data: %s", detection_data.c_str());

            // Search for the label "Cup" in the message
            if (detection_data.find("Cup") != std::string::npos)
            {
                // Use regular expression to extract label, center_x, center_y, and depth from the message
                std::regex detection_regex(R"(Label: (.+?) \| Center: \((\d+), (\d+)\) \| Depth: ([\d\.]+)m)");
                std::smatch match;
                if (std::regex_search(detection_data, match, detection_regex))
                {
                    std::string label = match[1];
                    int center_x = std::stoi(match[2]);
                    int center_y = std::stoi(match[3]);
                    float depth = std::stof(match[4]);

                    RCLCPP_INFO(this->get_logger(), "Found a Cup at (%d, %d) with depth %f meters.", center_x, center_y, depth);

                    // Convert the 2D coordinates to 3D coordinates using camera intrinsics
                    float x = (center_x - cx_) * depth / fx_;
                    float y = (center_y - cy_) * depth / fy_;
                    float z = depth; // Z is simply the depth value in meters

                    // Place the box in RViz
                    placeBoxInRviz(x, y, z);
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing detection data: %s", e.what());
        }
    }

    void placeBoxInRviz(float x, float y, float z)
    {
        moveit::planning_interface::CollisionObject collision_object;
        collision_object.id = "cup_box";
        collision_object.header.frame_id = "UR5_camera_link";  // Use your camera's TF frame

        // Define the box shape and its size
        shape_msgs::msg::SolidPrimitive box;
        box.type = box.BOX;
        box.dimensions = {0.1, 0.1, 0.1};  // Example box dimensions (width, height, depth)

        // Define the pose of the box
        geometry_msgs::msg::Pose box_pose;
        box_pose.position.x = x;
        box_pose.position.y = y;
        box_pose.position.z = z;

        // Add the box to the collision object
        collision_object.primitives.push_back(box);
        collision_object.primitive_poses.push_back(box_pose);

        // Add the collision object to the planning scene
        planning_scene_interface_->applyCollisionObject(collision_object);
        RCLCPP_INFO(this->get_logger(), "Placed a box at 3D coordinates: (%f, %f, %f)", x, y, z);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

    // Camera parameters (for RealSense D435i or other camera)
    float fx_, fy_, cx_, cy_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetectionListener>());
    rclcpp::shutdown();
    return 0;
}

