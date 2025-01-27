#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

// MoveIt
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <rviz_visual_tools/rviz_visual_tools.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("planning_scene_ros_api_tutorial");

int main(int argc, char** argv)
{




 rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("planning_scene_ros_api_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();
  // BEGIN_TUTORIAL
  //
  // Visualization
  // ^^^^^^^^^^^^^
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  rviz_visual_tools::RvizVisualTools visual_tools("ur5_arm", "planning_scene_ros_api_tutorial", node);
  visual_tools.loadRemoteControl();
  visual_tools.deleteAllMarkers();

  // ROS API
  // ^^^^^^^
  // The ROS API to the planning scene publisher is through a topic interface
  // using "diffs". A planning scene diff is the difference between the current
  // planning scene (maintained by the move_group node) and the new planning
  // scene desired by the user.
  //
  // Advertise the required topic
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // We create a publisher and wait for subscribers.
  // Note that this topic may need to be remapped in the launch file.
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher =
      node->create_publisher<moveit_msgs::msg::PlanningScene>("UR5/planning_scene", 1);
  while (planning_scene_diff_publisher->get_subscription_count() < 1)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  // Define the attached object message
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // We will use this message to add or
  // subtract the object from the world
  // and to attach the object to the robot.
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "UR5_camera_link";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "UR5_camera_link";
  /* The id of the object */
  attached_object.object.id = "box";

  /* A default pose */
  geometry_msgs::msg::Pose pose;
  pose.position.z = 0.5;
  pose.orientation.w = 1.0;

  /* Define a box to be attached */
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.1;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  // Note that attaching an object to the robot requires
  // the corresponding operation to be specified as an ADD operation.
  attached_object.object.operation = attached_object.object.ADD;

  // Since we are attaching the object to the robot hand to simulate picking up the object,
  // we want the collision checker to ignore collisions between the object and the robot hand.
  //attached_object.touch_links = std::vector<std::string>{ "panda_hand", "panda_leftfinger", "panda_rightfinger" };

  // Add an object into the environment
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Add the object into the environment by adding it to
  // the set of collision objects in the "world" part of the
  // planning scene. Note that we are using only the "object"
  // field of the attached_object message here.
  RCLCPP_INFO(LOGGER, "Adding the object into the world at the location of the Camera.");
  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher->publish(planning_scene);

  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr planning_scene_diff_client =
      node->create_client<moveit_msgs::srv::ApplyPlanningScene>("UR5/apply_planning_scene");
  planning_scene_diff_client->wait_for_service();
  // and send the diffs to the planning scene via a service call:
  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene = planning_scene;



  rclcpp::shutdown();
  return 0;

}
