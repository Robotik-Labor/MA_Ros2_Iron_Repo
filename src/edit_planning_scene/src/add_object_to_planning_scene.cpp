#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class AddObjectToPlanningScene : public rclcpp::Node
{
public:
  AddObjectToPlanningScene() : Node("add_object_to_planning_scene")
  {
    // Create the PlanningSceneInterface for the specific robot namespace
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>("/UR5");
    
    // Add a small delay to ensure the planning scene is ready
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // Add objects to the scene
    addObjectsToScene();
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
      object_pose.position.z = -0.015 + 0.01;   // Adjust as needed
      
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
    
    // Add all objects to the planning scene at once (more efficient)
    planning_scene_interface_->addCollisionObjects(collision_objects);
  }
  
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin_some(std::make_shared<AddObjectToPlanningScene>());
  rclcpp::shutdown();
  return 0;
}
