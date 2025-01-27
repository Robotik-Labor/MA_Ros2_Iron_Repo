#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>  // Thread for executor spin

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node (make sure node name is valid)
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "ur5_moveit_test", // Valid node name without slashes or invalid characters
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );


  // Create a ROS logger
  auto const logger = rclcpp::get_logger("ur5_moveit_test");

  // Executor setup
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Corrected MoveIt interface setup
//  moveit::planning_interface::MoveGroupInterface::Options options(
//    "gripper",               // The name of the MoveGroup
//    "robot_description", // The parameter for the robot's URDF (robot description)
//    "/UR5"                    // The namespace for MoveGroup (often for robot namespace)
//);

 moveit::planning_interface::MoveGroupInterface::Options options("gripper", "robot_description", "/UR5");
 auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, options); 


  // MoveIt Visual Tools setup
  moveit_visual_tools::MoveItVisualTools moveit_visual_tools(
      node, "UR5_base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel()
  );
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Set target pose and planning
  auto const target_pose = move_group_interface.getNamedTargetValues("open");
  move_group_interface.setJointValueTarget(target_pose);

  // Plan the motion
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::MoveItErrorCode error_code = move_group_interface.plan(plan);
  bool success = error_code == moveit::core::MoveItErrorCode::SUCCESS;

  if (success) {
    moveit_visual_tools.prompt("Planning Success! Start Executing?");
    move_group_interface.execute(plan);
  } else {
    moveit_visual_tools.prompt("Planning Failed!");
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();  // Join the spinner thread
  return 0;
}

