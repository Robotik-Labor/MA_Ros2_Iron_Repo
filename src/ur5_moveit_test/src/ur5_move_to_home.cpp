#include "rcutils/logging_macros.h"
#include <memory>
#include <string>
#include <vector>

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "rclcpp/rclcpp.hpp"
//#include "planning_scene_interface.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "std_msgs/msg/string.hpp"

int main(int argc, char **argv) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
      "ur5_move_to_home",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("ur5_move_to_home");

  auto cloud_message_publisher = node->create_publisher<std_msgs::msg::String>("/UR5/cloud_messages", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });
  // Create the MoveGroup interface for the UR5 arm
  using moveit::planning_interface::MoveGroupInterface;
  moveit::planning_interface::MoveGroupInterface::Options options(
      "ur5_arm", "robot_description", "/UR5");
      
  auto move_group_interface = MoveGroupInterface(node, options);

  const moveit::core::JointModelGroup *joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup("ur5_arm");  

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(
      node, "world", "/UR5/move_group_tutorial",
      move_group_interface.getRobotModel());
  visual_tools.deleteAllMarkers();

  // RViz provides many types of markers, in this demo we will use text,
  // cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Moving_to_Home", rvt::WHITE,
                           rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to
  // RViz for large visualizations
  visual_tools.trigger();

  // Allow time for the interfaces to initialize
  rclcpp::sleep_for(std::chrono::seconds(1));

  // Set planning parameters
  move_group_interface.clearPathConstraints();
  move_group_interface.setPlanningTime(20.0);
  move_group_interface.setMaxVelocityScalingFactor(0.1);
  move_group_interface.setMaxAccelerationScalingFactor(0.1);
  move_group_interface.setPlannerId("BiTRRT");
  move_group_interface.setNumPlanningAttempts(20);
  move_group_interface.setGoalJointTolerance(0.03);
  //move_group_interface.setWorkspace(-449, -200, -200, 850, 850, 1200);

  // moveit_msgs::msg::JointConstraint shoulder_lift_constraint;
  // shoulder_lift_constraint.joint_name = "UR5_shoulder_lift_joint";  // Name
  // of the joint shoulder_lift_constraint.position = -M_PI/2;  // Desired joint
  // position shoulder_lift_constraint.tolerance_above = M_PI/2;  // Tolerance
  // above the desired position shoulder_lift_constraint.tolerance_below =
  // M_PI/2;  // Tolerance below the desired position
  // shoulder_lift_constraint.weight = 1.0;  // Explicitly setting weight to 1.0
  // for importance

  // Create joint constraints for the wrist joints
  moveit_msgs::msg::JointConstraint wrist_1_constraint;
  wrist_1_constraint.joint_name = "UR5_wrist_1_joint"; // Name of the joint
  wrist_1_constraint.position = 0.0; // Center position (0 radians)
  wrist_1_constraint.tolerance_above = M_PI / 2; // ±180 degrees tolerance
  wrist_1_constraint.tolerance_below = M_PI / 2; // ±180 degrees tolerance
  wrist_1_constraint.weight = 0.8; // Explicitly setting weight to 1.0 for importance

  moveit_msgs::msg::JointConstraint wrist_2_constraint;
  wrist_2_constraint.joint_name = "UR5_wrist_2_joint"; // Name of the joint
  wrist_2_constraint.position = M_PI/2;         // Center position (0 radians)
  wrist_2_constraint.tolerance_above = M_PI/2; // ±180 degrees tolerance
  wrist_2_constraint.tolerance_below = M_PI/2; // ±180 degrees tolerance
  wrist_2_constraint.weight = 0.8; // Explicitly setting weight to 1.0 for importance

  moveit_msgs::msg::JointConstraint wrist_3_constraint;
  wrist_3_constraint.joint_name = "UR5_wrist_3_joint"; // Name of the joint
  wrist_3_constraint.position = M_PI;         // Center position (0 radians)
  wrist_3_constraint.tolerance_above = M_PI; // ±180 degrees tolerance
  wrist_3_constraint.tolerance_below = M_PI; // ±180 degrees tolerance
  wrist_3_constraint.weight = 0.8; // Explicitly setting weight to 1.0 for importance

  moveit_msgs::msg::Constraints constraints;
  // constraints.joint_constraints.push_back(shoulder_lift_constraint);
  constraints.joint_constraints.push_back(wrist_1_constraint);
  constraints.joint_constraints.push_back(wrist_2_constraint);
  constraints.joint_constraints.push_back(wrist_3_constraint);
  move_group_interface.setPathConstraints(constraints);


  move_group_interface.setNamedTarget("Home");

  visual_tools.publishText(text_pose, "Calculating_Path_Home", rvt::WHITE,
                           rvt::XLARGE);
  visual_tools.trigger(); 


  moveit::planning_interface::MoveGroupInterface::Plan plan_home;
  bool success = (move_group_interface.plan(plan_home) ==
             moveit::core::MoveItErrorCode::SUCCESS);
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  RCLCPP_INFO(logger, "Visualizing Home trajectory line");
  // visual_tools.publishAxisLabeled(pre_grasp_pose, "pre-grasp-position");
  visual_tools.publishText(text_pose, "Moving_to_Home", rvt::WHITE,
                           rvt::XLARGE);
  visual_tools.publishTrajectoryLine(plan_home.trajectory, joint_model_group);
  visual_tools.trigger();
  
  move_group_interface.execute(plan_home);
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();


  // Create and publish the completion message
  std_msgs::msg::String msg;
  msg.data = "Move_to_Home_Complete";
  cloud_message_publisher->publish(msg);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
