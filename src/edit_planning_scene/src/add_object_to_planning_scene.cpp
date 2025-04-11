#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/string.hpp>  // Added for String message type

class AddObjectToPlanningScene : public rclcpp::Node
{
public:
  AddObjectToPlanningScene() : Node("add_object_to_planning_scene")
  {
    // Create the PlanningSceneInterface for the specific robot namespace
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>("/UR5");
    
    scene_loaded_publisher_ = this->create_publisher<std_msgs::msg::String>(
  	"/UR5/cloud_messages", 
  	10  // QoS profile depth
	);
    
    // Add a small delay to ensure the planning scene is ready
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // Add objects to the scene
    addObjectsToScene();
    publishSceneLoaded();
  }

private:
  // Helper function to convert RPY angles to quaternion
  geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();
    return q_msg;
  }
  
  // Helper function to create a mesh collision object
  moveit_msgs::msg::CollisionObject createMeshObject(
      const std::string& id, 
      const std::string& mesh_path, 
      const geometry_msgs::msg::Pose& pose)
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = id;
    collision_object.header.frame_id = "world";
    
    // Load the mesh from the STL file
    shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path);
    if (mesh) {
      // Convert the shapes::Mesh to shape_msgs::Mesh
      shapes::ShapeMsg mesh_msg;
      shapes::constructMsgFromShape(mesh, mesh_msg);
      
      // Add the mesh to the collision object
      collision_object.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(mesh_msg));
      collision_object.mesh_poses.push_back(pose);
      collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
      
      delete mesh;  // Clean up the mesh object
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to load mesh from %s", mesh_path.c_str());
    }
    
    return collision_object;
  }
  
  
  void publishSceneLoaded()
	{
  auto message = std_msgs::msg::String();
  message.data = "Scene_Loaded";
  scene_loaded_publisher_->publish(message);
  RCLCPP_INFO(this->get_logger(), "Published Scene_Loaded message");
	}
  
  void addObjectsToScene()
  {
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    
    // First mesh (Sensor Station)
    {
      geometry_msgs::msg::Pose object_pose;
      object_pose.position.x = 0.42;
      object_pose.position.y = -0.18;
      object_pose.position.z = -0.015 + 0.0001;
      
      double roll = 1.5708;   // rotation around x-axis (90 degrees)
      double pitch = 0.0;     // rotation around y-axis
      double yaw = 1.5708;    // rotation around z-axis (90 degrees)
      object_pose.orientation = rpy_to_quaternion(roll, pitch, yaw);
      
      std::string stl_path = "package://ur5_description_controller/meshes/sensor_dummy/Sensor_Halterung.stl";
      
      moveit_msgs::msg::CollisionObject sensor_station = createMeshObject("Sensor_Station", stl_path, object_pose);
      collision_objects.push_back(sensor_station);
      
      RCLCPP_INFO(this->get_logger(), "Added Sensor_Station with orientation: Roll=%f, Pitch=%f, Yaw=%f", 
                 roll, pitch, yaw);
    }
    
    // Second mesh (add your new mesh here)
    {
      geometry_msgs::msg::Pose object_pose;
      object_pose.position.x = 0.42;   // Adjust as needed
      object_pose.position.y = -0.18;   // Adjust as needed
      object_pose.position.z = -0.015 + 0.003;   // Adjust as needed
      
      double roll = 1.5708;
      double pitch = 0.0;
      double yaw = 1.5708;
      object_pose.orientation = rpy_to_quaternion(roll, pitch, yaw);
      
      // Change this to the path of your second mesh
      std::string stl_path = "package://ur5_description_controller/meshes/sensor_dummy/Sensor.stl";
      
      moveit_msgs::msg::CollisionObject second_object = createMeshObject("Sensor", stl_path, object_pose);
      collision_objects.push_back(second_object);
      
      RCLCPP_INFO(this->get_logger(), "Added Second_Object with orientation: Roll=%f, Pitch=%f, Yaw=%f", 
                 roll, pitch, yaw);
    }
    
    // Plane behind the robot
{
    geometry_msgs::msg::Pose object_pose;
    object_pose.position.x = 0.00;   // Updated position
    object_pose.position.y = -0.51;  // Updated position
    object_pose.position.z = 0.54 + 0.0001;   // Updated position
    
    // Setting orientation to identity (w=1)
    object_pose.orientation.x = 0.0;
    object_pose.orientation.y = 0.0;
    object_pose.orientation.z = 0.0;
    object_pose.orientation.w = 1.0;
    
    // Create a box collision object instead of using mesh
    moveit_msgs::msg::CollisionObject box_object;
    box_object.header.frame_id = "world";  // Assuming base_link is your reference frame
    box_object.id = "SafetyPlane_Back";
    
    // Define box dimensions
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.90;  // Box x dimension
    primitive.dimensions[1] = 0.01;  // Box y dimension
    primitive.dimensions[2] = 1.20;  // Box z dimension
    
    box_object.primitives.push_back(primitive);
    box_object.primitive_poses.push_back(object_pose);
    box_object.operation = box_object.ADD;
    
    collision_objects.push_back(box_object);
    
     RCLCPP_INFO(this->get_logger(), "Added Box with dimensions: x=%f, y=%f, z=%f at position: x=%f, y=%f, z=%f", 
                primitive.dimensions[0], primitive.dimensions[1], primitive.dimensions[2],
                object_pose.position.x, object_pose.position.y, object_pose.position.z);
 }
  
    // Touchpad Safetyplane
{
    geometry_msgs::msg::Pose object_pose;
    object_pose.position.x = -0.375 ;   // Updated position
    object_pose.position.y = -0.31;  // Updated position
    object_pose.position.z = 0.28 +0.0001;   // Updated position
    
    // Setting orientation to identity (w=1)
    object_pose.orientation.x = 0.0;
    object_pose.orientation.y = 0.0;
    object_pose.orientation.z = 0.0;
    object_pose.orientation.w = 1.0;
    
    // Create a box collision object instead of using mesh
    moveit_msgs::msg::CollisionObject box_object;
    box_object.header.frame_id = "world";  // Assuming base_link is your reference frame
    box_object.id = "SafetyPlane_Touchpad";
    
    // Define box dimensions
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.15;  // Box x dimension
    primitive.dimensions[1] = 0.40;  // Box y dimension
    primitive.dimensions[2] = 0.60;  // Box z dimension
    
    box_object.primitives.push_back(primitive);
    box_object.primitive_poses.push_back(object_pose);
    box_object.operation = box_object.ADD;
    
    collision_objects.push_back(box_object);
    
     RCLCPP_INFO(this->get_logger(), "Added Box with dimensions: x=%f, y=%f, z=%f at position: x=%f, y=%f, z=%f", 
                primitive.dimensions[0], primitive.dimensions[1], primitive.dimensions[2],
                object_pose.position.x, object_pose.position.y, object_pose.position.z);
 }
  
    // Site Safetyplane
{
    geometry_msgs::msg::Pose object_pose;
    object_pose.position.x = -0.456 - 0.0001;   // Updated position
    object_pose.position.y = 0.48;  // Updated position
    object_pose.position.z = 0.54;   // Updated position
    
    // Setting orientation to identity (w=1)
    object_pose.orientation.x = 0.0;
    object_pose.orientation.y = 0.0;
    object_pose.orientation.z = 0.0;
    object_pose.orientation.w = 1.0;
    
    // Create a box collision object instead of using mesh
    moveit_msgs::msg::CollisionObject box_object;
    box_object.header.frame_id = "world";  // Assuming base_link is your reference frame
    box_object.id = "SafetyPlane_Side";
    
    // Define box dimensions
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.01;  // Box x dimension
    primitive.dimensions[1] = 2.00;  // Box y dimension
    primitive.dimensions[2] = 1.20;  // Box z dimension
    
    box_object.primitives.push_back(primitive);
    box_object.primitive_poses.push_back(object_pose);
    box_object.operation = box_object.ADD;
    
    
    
    collision_objects.push_back(box_object);
    
     RCLCPP_INFO(this->get_logger(), "Added Box with dimensions: x=%f, y=%f, z=%f at position: x=%f, y=%f, z=%f", 
                primitive.dimensions[0], primitive.dimensions[1], primitive.dimensions[2],
                object_pose.position.x, object_pose.position.y, object_pose.position.z);
 }
  
      // Conveyor belt
{
    geometry_msgs::msg::Pose object_pose;
    object_pose.position.x = 0.76;   // Updated position
    object_pose.position.y = 0.0;  // Updated position
    object_pose.position.z = 0.07;   // Updated position
    
    // Setting orientation to identity (w=1)
    object_pose.orientation.x = 0.0;
    object_pose.orientation.y = 0.0;
    object_pose.orientation.z = 0.0;
    object_pose.orientation.w = 1.0;
    
    // Create a box collision object instead of using mesh
    moveit_msgs::msg::CollisionObject box_object;
    box_object.header.frame_id = "world";  // Assuming base_link is your reference frame
    box_object.id = "Conveyor_Belt";
    
    // Define box dimensions
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.40;  // Box x dimension
    primitive.dimensions[1] = 0.7;  // Box y dimension
    primitive.dimensions[2] = 0.25;  // Box z dimension
    
    box_object.primitives.push_back(primitive);
    box_object.primitive_poses.push_back(object_pose);
    box_object.operation = box_object.ADD;
    
    
    
    collision_objects.push_back(box_object);
    
     RCLCPP_INFO(this->get_logger(), "Added Box with dimensions: x=%f, y=%f, z=%f at position: x=%f, y=%f, z=%f", 
                primitive.dimensions[0], primitive.dimensions[1], primitive.dimensions[2],
                object_pose.position.x, object_pose.position.y, object_pose.position.z);
 }
  
  
      // Add all objects to the planning scene at once (more efficient)
    planning_scene_interface_->addCollisionObjects(collision_objects);
    
    // Set color for the box using a planning scene update
	moveit_msgs::msg::PlanningScene planning_scene;
	planning_scene.is_diff = true;

	// Add color information
	moveit_msgs::msg::ObjectColor box_color;
	box_color.id = "SafetyPlane_Back";
	box_color.color.r = 0.64;   // Red component (0-1)
	box_color.color.g = 0.76;   // Green component (0-1)
	box_color.color.b = 0.92;   // Blue component (0-1)
	box_color.color.a = 1.0;   // Alpha (transparency) (0-1)
	planning_scene.object_colors.push_back(box_color);
	
	// Add color information
	box_color.id = "SafetyPlane_Touchpad";
	box_color.color.r = 0.64;   // Red component (0-1)
	box_color.color.g = 0.76;   // Green component (0-1)
	box_color.color.b = 0.92;   // Blue component (0-1)
	box_color.color.a = 1.0;   // Alpha (transparency) (0-1)
	planning_scene.object_colors.push_back(box_color);
	
	// Add color information
	box_color.id = "SafetyPlane_Side";
	box_color.color.r = 0.64;   // Red component (0-1)
	box_color.color.g = 0.76;   // Green component (0-1)
	box_color.color.b = 0.92;   // Blue component (0-1)
	box_color.color.a = 1.0;   // Alpha (transparency) (0-1)
	planning_scene.object_colors.push_back(box_color);
	
        // Add color information
	box_color.id = "Conveyor_Belt";
	box_color.color.r = 0.64;   // Red component (0-1)
	box_color.color.g = 0.76;   // Green component (0-1)
	box_color.color.b = 0.92;   // Blue component (0-1)
	box_color.color.a = 1.0;   // Alpha (transparency) (0-1)
	planning_scene.object_colors.push_back(box_color);

        // Apply the planning scene to set the color
	planning_scene_interface_->applyPlanningScene(planning_scene);
    
  }
  
  
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr scene_loaded_publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin_some(std::make_shared<AddObjectToPlanningScene>());
  rclcpp::shutdown();
  return 0;
}
