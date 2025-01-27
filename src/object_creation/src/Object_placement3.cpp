#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

        RCLCPP_INFO(this->get_logger(), "Ready to listen to detections and place objects in RViz.");
        
        // Initialize tf2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Declare publisher for coordinates above the cup
        above_cup_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/UR5/AboveCup", 10);

        // No need to call getTransformation here, since it's called later in placeBoxInRviz()
    }

private:
    // Add the publisher as a member
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;
    
    // Add subscription and transform listener as members
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Add publisher for the coordinates above the cup
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr above_cup_publisher_;

   void detectionCallback(const std_msgs::msg::String::SharedPtr msg)
	{
	    try
	    {
		std::string detection_data = msg->data;
		RCLCPP_INFO(this->get_logger(), "Received detection data: %s", detection_data.c_str());

		std::regex detection_regex(R"(Label: (\w+) \| Coordinates: \(([-+]?\d*\.\d+), ([-+]?\d*\.\d+), ([-+]?\d*\.\d+)\) \|)");
		std::smatch match;

		// Flag to track if a cup is detected
		bool cup_detected = false;

		// Search for all matches in the input string
		auto words_begin = std::sregex_iterator(detection_data.begin(), detection_data.end(), detection_regex);
		auto words_end = std::sregex_iterator();

		for (std::sregex_iterator i = words_begin; i != words_end; ++i) {
		    std::smatch match = *i;
		    std::string label = match[1];  // Extract label (e.g., "keyboard", "cup", "tv")
		    
		    // Check if the label is "cup"
		    if (label == "cup") {
		        cup_detected = true;
		        float x = std::stof(match[2]);  // x-coordinate
		        float y = std::stof(match[3]);  // y-coordinate
		        float z = std::stof(match[4]);  // z-coordinate

		        // Log and use the coordinates of the "cup"
		        RCLCPP_INFO(this->get_logger(), "Found a Cup at (%f, %f, %f)", x, y, z);
		        placeBoxInRviz(x, y, z);
		    }
		}

		// If no cup was detected, delete the box
		if (!cup_detected) {
		    RCLCPP_INFO(this->get_logger(), "No cup detected. Deleting the box.");
		    deleteBoxFromRviz();
		}
	    }
	    catch (const std::exception &e)
	    {
		RCLCPP_ERROR(this->get_logger(), "Error processing detection data: %s", e.what());
	    }
	}

	void deleteBoxFromRviz()
	{
	    // Create a PlanningScene to remove the box from the planning scene
	    moveit_msgs::msg::PlanningScene planning_scene;

	    moveit_msgs::msg::AttachedCollisionObject attached_object;
	    attached_object.object.id = "cup_box";  // ID of the box to delete
	    attached_object.object.operation = attached_object.object.REMOVE;  // Set operation to REMOVE

	    // Add the removal action to the planning scene
	    planning_scene.world.collision_objects.push_back(attached_object.object);
	    planning_scene.is_diff = true;  // Indicate it's a differential update (change)

	    // Publish the updated planning scene
	    planning_scene_diff_publisher_->publish(planning_scene);
	    RCLCPP_INFO(this->get_logger(), "Box removed from RViz and planning scene.");
	}

    void placeBoxInRviz(float x, float y, float z)
    {
        // Get the transformation between "UR5_base_link" and "camera_link"
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform(
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

            // Convert the object position into a geometry_msgs::Point
            geometry_msgs::msg::Point object_position;
            object_position.x = x;
            object_position.y = y;
            object_position.z = z;

            // Apply the transformation (translation + rotation) using tf2::doTransform
            geometry_msgs::msg::Point transformed_position;
            tf2::doTransform(object_position, transformed_position, transform);

            // Now, transformed_position contains the transformed coordinates in the target frame

            // Define the pose of the box
            geometry_msgs::msg::Pose box_pose;
            box_pose.position.x = transformed_position.x;
            box_pose.position.y = transformed_position.y;
            box_pose.position.z = transformed_position.z;

            // Set the box orientation using the rotation from the transform (instead of manual quaternions)
            //box_pose.orientation = transform.transform.rotation;
            box_pose.orientation.x = 0.0;
            box_pose.orientation.y = 0.0;
            box_pose.orientation.z = 0.0;
            box_pose.orientation.w = 1.0;
	    
            // Define the box properties as before
            moveit_msgs::msg::PlanningScene planning_scene;
            moveit_msgs::msg::AttachedCollisionObject attached_object;
            attached_object.link_name = "UR5_camera_link";  // Use the correct link name
            attached_object.object.header.frame_id = "UR5_base_link";  // Frame for object placement
            attached_object.object.id = "cup_box";

            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.1;  // Box width
            primitive.dimensions[1] = 0.1;  // Box height
            primitive.dimensions[2] = 0.1;  // Box depth

            attached_object.object.primitives.push_back(primitive);
            attached_object.object.primitive_poses.push_back(box_pose);
            attached_object.object.operation = attached_object.object.ADD;

            planning_scene.world.collision_objects.push_back(attached_object.object);
            planning_scene.is_diff = true;

            planning_scene_diff_publisher_->publish(planning_scene);
            RCLCPP_INFO(this->get_logger(), "Placed a box at transformed 3D coordinates: (%f, %f, %f)", box_pose.position.x, box_pose.position.y, box_pose.position.z);

            // Publish the coordinates of the placed box, but with z + 40 cm
            geometry_msgs::msg::Point above_cup_position;
            above_cup_position.x = transformed_position.x;
            above_cup_position.y = transformed_position.y;
            above_cup_position.z = transformed_position.z + 0.4;  // Add 40 cm to the z-coordinate

            above_cup_publisher_->publish(above_cup_position);
            RCLCPP_INFO(this->get_logger(), "Published position above the cup: (%f, %f, %f)", above_cup_position.x, above_cup_position.y, above_cup_position.z);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }
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

