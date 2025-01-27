#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>   // For trigonometrical functions
#include <vector>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "ur5_move_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("ur5_moveit_test");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  moveit::planning_interface::MoveGroupInterface::Options options("ur5_arm", "robot_description", "/UR5");
  auto move_group_interface = MoveGroupInterface(node, options);

  // Store the 5 positions
  std::vector<geometry_msgs::msg::Pose> target_poses;

  // Subscriber to /UR5/AboveCup topic to calculate positions
  auto subscription = node->create_subscription<geometry_msgs::msg::Point>(
    "/UR5/AboveCup", 10, [&move_group_interface, &logger, &target_poses](const geometry_msgs::msg::Point::SharedPtr msg) {
      RCLCPP_INFO(logger, "Received point: x=%.2f, y=%.2f, z=%.2f", msg->x, msg->y, msg->z);
      
      // Center of the semicircle
      double center_x = msg->x;
      double center_y = msg->y;
      double center_z = msg->z;

      // Define the radius of the semicircle
      const double radius = 0.2;
      const int num_positions = 5;
      const double angle_step = M_PI / (num_positions - 1);  // Divide the semicircle into equal parts

      // Calculate positions and store them
      for (int i = 0; i < num_positions; ++i) {
        double angle = i * angle_step;  // Angle for current position
        double x = center_x + radius * cos(angle);
        double y = center_y + radius * sin(angle);

        // Define target pose
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.x = 0.89773553;
        target_pose.orientation.y = -0.0003130799;
        target_pose.orientation.z = -0.0007278507;
        target_pose.orientation.w = -0.4405340850;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = center_z;

        target_poses.push_back(target_pose);
      }

      RCLCPP_INFO(logger, "Positions calculated, moving to target positions...");
    });

  // Ensure the program waits until the positions are calculated
  while (target_poses.empty()) {
    rclcpp::spin_some(node);  // Wait for the subscription callback to fill target_poses
  }

  // Move the robot to each position
  for (const auto& target_pose : target_poses) {
    bool success = false;
    int attempts = 0;

    // Retry mechanism: attempt up to 3 times
    while (attempts < 3 && !success) {
      move_group_interface.setPoseTarget(target_pose);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group_interface.plan(plan)) {
        RCLCPP_INFO(logger, "Planning Success! Executing...");
        moveit::core::MoveItErrorCode result = move_group_interface.execute(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(logger, "Successfully moved to position: (%.2f, %.2f, %.2f)", target_pose.position.x, target_pose.position.y, target_pose.position.z);
          success = true;  // Exit loop on success
        } else {
          RCLCPP_ERROR(logger, "Failed to move to position. Attempt %d/3. Error code: %d", attempts + 1, result.val);
        }
      } else {
        RCLCPP_ERROR(logger, "Planning failed for position: (%.2f, %.2f, %.2f). Attempt %d/3", target_pose.position.x, target_pose.position.y, target_pose.position.z, attempts + 1);
      }
      ++attempts;
    }

    // If after 3 attempts, the move failed, log a message and proceed to the next position
    if (!success) {
      RCLCPP_ERROR(logger, "Failed to reach position after 3 attempts: (%.2f, %.2f, %.2f)", target_pose.position.x, target_pose.position.y, target_pose.position.z);
    }

    // Perform some function at each position (this is a placeholder)
    RCLCPP_INFO(logger, "Performing action at position: (%.2f, %.2f, %.2f)", target_pose.position.x, target_pose.position.y, target_pose.position.z);
  }

  // Return to the original position (Assuming home pose is at x=0, y=0, z=0.5)
  geometry_msgs::msg::Pose home_pose;
  home_pose.position.x = 0.0;
  home_pose.position.y = 0.0;
  home_pose.position.z = 0.5;
  home_pose.orientation.w = 1.0;

  move_group_interface.setPoseTarget(home_pose);
  moveit::planning_interface::MoveGroupInterface::Plan return_plan;
  if (move_group_interface.plan(return_plan)) {
    RCLCPP_INFO(logger, "Planning to return to home position...");

    // Check the result of execute
    moveit::core::MoveItErrorCode result = move_group_interface.execute(return_plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "Successfully returned to home position.");
    } else {
      RCLCPP_ERROR(logger, "Failed to return to home position. Error code: %d", result.val);
    }
  } else {
    RCLCPP_ERROR(logger, "Failed to plan return to home position.");
  }

  // Shutdown ROS after the task is finished
  rclcpp::shutdown();
  return 0;
}
