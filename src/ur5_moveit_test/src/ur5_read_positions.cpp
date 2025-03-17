#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "ur5_read_state_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("ur5_moveit_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  moveit::planning_interface::MoveGroupInterface::Options options("ur5_arm", "robot_description", "/UR5");
  auto move_group_interface = MoveGroupInterface(node, options);

  // Get current joint values
  auto joint_values = move_group_interface.getCurrentJointValues();
  RCLCPP_INFO(logger, "Current Joint Values:");
  for (size_t i = 0; i < joint_values.size(); ++i) {
    RCLCPP_INFO(logger, "Joint %zu: %.2f", i, joint_values[i]);
  }

  // Get current end-effector (tool) position
  auto end_effector_pose = move_group_interface.getCurrentPose().pose;
  RCLCPP_INFO(logger, "End-Effector Position:");
  RCLCPP_INFO(logger, "Position: x=%.2f, y=%.2f, z=%.2f", end_effector_pose.position.x, end_effector_pose.position.y, end_effector_pose.position.z);
  RCLCPP_INFO(logger, "Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f", end_effector_pose.orientation.x, end_effector_pose.orientation.y, end_effector_pose.orientation.z, end_effector_pose.orientation.w);

  geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose("UR5_tool0");
  RCLCPP_INFO(logger, "Current end effector pose: x=%.3f, y=%.3f, z=%.3f", 
              current_pose.pose.position.x, 
              current_pose.pose.position.y, 
              current_pose.pose.position.z);
  
 
  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();  // Join the thread before exiting

  return 0;
}

