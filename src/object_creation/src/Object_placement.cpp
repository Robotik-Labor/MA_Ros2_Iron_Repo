#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <regex>
#include <string>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("object_creation");

class ObjectDetectionListener : public rclcpp::Node
{
public:
    ObjectDetectionListener(rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher)
        : Node("object_detection_listener"), planning_scene_diff_publisher_(planning_scene_diff_publisher)
    {
        // Subscribe to the detected object topic
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "UR5/detected_objects", 10, std::bind(&ObjectDetectionListener::detectionCallback, this, std::placeholders::_1));

        // Camera parameters (for RealSense D435i or other camera)
        fx_ = 907.4872436523438;  // Focal length in x direction (in pixels)
        fy_ = 907.2102661132812;  // Focal length in y direction (in pixels)
        cx_ = 651.9036254882812;     // Principal point (x)
        cy_ = 371.484375;     // Principal point (y)

        RCLCPP_INFO(this->get_logger(), "Ready to listen to detections and place objects in RViz.");
    }

private:
    void detectionCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        try
        {
            std::string detection_data = msg->data;
            RCLCPP_INFO(this->get_logger(), "Received detection data: %s", detection_data.c_str());

		std::regex detection_regex(R"(Label: (\w+) \| Coordinates: \(([-+]?\d*\.\d+), ([-+]?\d*\.\d+), ([-+]?\d*\.\d+)\) \|)");
		std::smatch match;

		// Search for all matches in the input string
		auto words_begin = std::sregex_iterator(detection_data.begin(), detection_data.end(), detection_regex);
		auto words_end = std::sregex_iterator();

		for (std::sregex_iterator i = words_begin; i != words_end; ++i) {
		    std::smatch match = *i;
		    std::string label = match[1];  // Extract label (e.g., "keyboard", "cup", "tv")
		    
		    // Check if the label is "cup"
		    if (label == "cup") {
			float x = std::stof(match[2]);  // x-coordinate
			float y = std::stof(match[3]);  // y-coordinate
			float z = std::stof(match[4]);  // z-coordinate

			// Log and use the coordinates of the "cup"
			RCLCPP_INFO(this->get_logger(), "Found a Cup at (%f, %f, %f)", x, y, z);
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
        moveit_msgs::msg::PlanningScene planning_scene;
        moveit_msgs::msg::AttachedCollisionObject attached_object;
        attached_object.link_name = "UR5_camera_link";  // Use the correct link name
        attached_object.object.header.frame_id = "UR5_camera_link";  // Use your camera's TF frame
        attached_object.object.id = "cup_box";

        // Define a box to be added into the planning scene
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.1;  // Box width
        primitive.dimensions[1] = 0.1;  // Box height
        primitive.dimensions[2] = 0.1;  // Box depth

        // Define the pose of the box based on the detected position
        geometry_msgs::msg::Pose box_pose;
        box_pose.position.x = x;
        box_pose.position.y = y;
        box_pose.position.z = z;

        // Add the box to the attached object
        attached_object.object.primitives.push_back(primitive);
        attached_object.object.primitive_poses.push_back(box_pose);
        attached_object.object.operation = attached_object.object.ADD;

        // Publish the planning scene update
        planning_scene.world.collision_objects.push_back(attached_object.object);
        planning_scene.is_diff = true;

        // Publish the planning scene to MoveIt!
        planning_scene_diff_publisher_->publish(planning_scene);
        RCLCPP_INFO(this->get_logger(), "Placed a box at 3D coordinates: (%f, %f, %f)", x, y, z);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;
    
    // Camera parameters (for RealSense D435i or other camera)
    float fx_, fy_, cx_, cy_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("object_detection_listener", node_options);

    // Planning Scene Publisher
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher =
      node->create_publisher<moveit_msgs::msg::PlanningScene>("UR5/planning_scene", 1);

    // Wait for the publisher to be ready
    while (planning_scene_diff_publisher->get_subscription_count() < 1)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    // Create and spin the node
    auto object_detection_listener = std::make_shared<ObjectDetectionListener>(planning_scene_diff_publisher);
    rclcpp::spin(object_detection_listener);

    rclcpp::shutdown();
    return 0;
}
