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
      "sensor_drop_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("sensor_drop_node");

  auto cloud_message_publisher = node->create_publisher<std_msgs::msg::String>("/UR5/cloud_messages", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });
  // Create the MoveGroup interface for the UR5 arm
  using moveit::planning_interface::MoveGroupInterface;
  moveit::planning_interface::MoveGroupInterface::Options options(
      "ur5_arm", "robot_description", "/UR5");
  auto move_group_interface = MoveGroupInterface(node, options);

  // Create the MoveGroup interface for the gripper
  moveit::planning_interface::MoveGroupInterface::Options gripper_options(
      "gripper", "robot_description", "/UR5");
  moveit::planning_interface::MoveGroupInterface gripper_group(node,
                                                               gripper_options);

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
  visual_tools.publishText(text_pose, "Retrieving_Sensor_Demo", rvt::WHITE,
                           rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to
  // RViz for large visualizations
  visual_tools.trigger();

  // Allow time for the interfaces to initialize
  rclcpp::sleep_for(std::chrono::seconds(1));

  // Set planning parameters
  move_group_interface.setPlanningTime(20.0);
  move_group_interface.setMaxVelocityScalingFactor(0.1);
  move_group_interface.setMaxAccelerationScalingFactor(0.1);
  move_group_interface.setPlannerId("BiTRRT");
  move_group_interface.setNumPlanningAttempts(20);
  move_group_interface.setGoalJointTolerance(0.03);
  move_group_interface.setWorkspace(-449, -200, -200, 850, 850, 1200);

  gripper_group.setMaxVelocityScalingFactor(0.1);
  gripper_group.setMaxAccelerationScalingFactor(0.1);

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
  wrist_2_constraint.tolerance_above = M_PI / 2; // ±180 degrees tolerance
  wrist_2_constraint.tolerance_below = M_PI / 2; // ±180 degrees tolerance
  wrist_2_constraint.weight = 0.8; // Explicitly setting weight to 1.0 for importance

  moveit_msgs::msg::JointConstraint wrist_3_constraint;
  wrist_3_constraint.joint_name = "UR5_wrist_3_joint"; // Name of the joint
  wrist_3_constraint.position = M_PI;         // Center position (0 radians)
  wrist_3_constraint.tolerance_above = M_PI/2; // ±180 degrees tolerance
  wrist_3_constraint.tolerance_below = M_PI/2; // ±180 degrees tolerance
  wrist_3_constraint.weight = 0.8; // Explicitly setting weight to 1.0 for importance

  moveit_msgs::msg::Constraints constraints;
  // constraints.joint_constraints.push_back(shoulder_lift_constraint);
  constraints.joint_constraints.push_back(wrist_1_constraint);
  constraints.joint_constraints.push_back(wrist_2_constraint);
  constraints.joint_constraints.push_back(wrist_3_constraint);
  move_group_interface.setPathConstraints(constraints);

  // Get and set the reference frame
  std::string reference_frame = move_group_interface.getPlanningFrame();
  RCLCPP_INFO(logger, "Using reference frame: %s", reference_frame.c_str());
  std::string pose_reference_frame =
      move_group_interface.getPoseReferenceFrame();
  RCLCPP_INFO(logger, "Using reference pose_reference_frame: %s",
              pose_reference_frame.c_str());
  move_group_interface.setPoseReferenceFrame(pose_reference_frame);
  // RCLCPP_INFO(logger, "Using reference frame: %s", reference_frame.c_str());

  // Define the target pose for the sensor object
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.241;
  target_pose.position.y = -0.18;
  target_pose.position.z = 0.09;
  target_pose.orientation.x = 0.50;
  target_pose.orientation.y = -0.50;
  target_pose.orientation.z = 0.50;
  target_pose.orientation.w = -0.50;

  // Create a pre-grasp pose with 5cm offset along x-axis
  geometry_msgs::msg::Pose pre_grasp_pose = target_pose;
  pre_grasp_pose.position.z += 0.1; // 5cm offset along x-axis

  // Create a post-grasp pose with 7cm offset along z-axis
  geometry_msgs::msg::Pose post_grasp_pose = target_pose;
  post_grasp_pose.position.z += 0.1; // 7cm offset along z-axis

  // Get the current pose to use as starting point
  geometry_msgs::msg::PoseStamped current_pose =
      move_group_interface.getCurrentPose("UR5_tool0");
  RCLCPP_INFO(logger, "Current end effector pose: x=%.3f, y=%.3f, z=%.3f",
              current_pose.pose.position.x, current_pose.pose.position.y,
              current_pose.pose.position.z);

  // Open the gripper
  RCLCPP_INFO(logger, "Opening gripper");
  gripper_group.setNamedTarget("open");
  gripper_group.move();

  // Move to pre-grasp position
  RCLCPP_INFO(logger, "Moving to pre-grasp position");
  move_group_interface.setPoseTarget(pre_grasp_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_interface.plan(plan) ==
                  moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(logger, "Visualizing pre-grasp-position trajectory line");
  visual_tools.publishAxisLabeled(pre_grasp_pose, "pre-grasp-position");
  visual_tools.publishText(text_pose, "Moving_to_Pick_Up", rvt::WHITE,
                           rvt::XLARGE);
  visual_tools.publishTrajectoryLine(plan.trajectory, joint_model_group);
  visual_tools.trigger();

  if (success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning to pre-grasp position failed!");
    return 1;
  }

  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  // Get the end effector link to use for cartesian path planning
  std::string end_effector_link = move_group_interface.getEndEffectorLink();
  RCLCPP_INFO(logger, "End effector link: %s", end_effector_link.c_str());

  // Move linearly from pre-grasp to grasp position
  RCLCPP_INFO(logger, "Moving linearly to grasp position");

  // Compute a Cartesian path for linear movement
  std::vector<geometry_msgs::msg::Pose> waypoints;

  // Start from the current position (very important for cartesian path
  // planning)
  current_pose = move_group_interface.getCurrentPose(end_effector_link);
  RCLCPP_INFO(logger,
              "getCurrentPose: Position [%.2f, %.2f, %.2f], Orientation [%.2f, "
              "%.2f, %.2f, %.2f], Frame: %s",
              current_pose.pose.position.x, current_pose.pose.position.y,
              current_pose.pose.position.z, current_pose.pose.orientation.x,
              current_pose.pose.orientation.y, current_pose.pose.orientation.z,
              current_pose.pose.orientation.w,
              current_pose.header.frame_id.c_str());
  geometry_msgs::msg::Pose start_pose = current_pose.pose;

  // Try to compute a cartesian path
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.005; // 1cm resolution

  // Get current pose before grasping
  current_pose = move_group_interface.getCurrentPose(end_effector_link);
  start_pose = current_pose.pose;

  // Set the target pose for lifting (7cm in z direction)
  geometry_msgs::msg::Pose grab_waypoint = start_pose;
  grab_waypoint.position.z -= 0.07; // 7cm up

  waypoints.push_back(grab_waypoint);

  double fraction = move_group_interface.computeCartesianPath(
      waypoints, eef_step, trajectory);

  RCLCPP_INFO(logger, "Cartesian path fraction: %.2f", fraction);

  if (fraction > 0.0) {
    // Execute the lifting path
    plan.trajectory = trajectory;

    RCLCPP_INFO(logger, "Visualizing Moving Down trajectory");
    visual_tools.publishAxisLabeled(grab_waypoint, "grasp-position");
    visual_tools.publishText(text_pose, "Linear_Down", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory, joint_model_group);
    visual_tools.trigger();

    move_group_interface.execute(plan);
    RCLCPP_INFO(logger, "Linear lifting executed (%.1f%% of path achieved)",
                fraction * 100.0);
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
  } else {
    move_group_interface.clearPathConstraints();
    RCLCPP_ERROR(logger, "Failed to compute cartesian path for lifting. "
                         "Falling back to regular planning.");
    move_group_interface.setPoseTarget(target_pose);
    success = (move_group_interface.plan(plan) ==
               moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      move_group_interface.execute(plan);
    } else {
      RCLCPP_ERROR(logger, "Planning to post-grasp position failed!");
      return 1;
    }
  }

  // Close the gripper to grasp the sensor
  RCLCPP_INFO(logger, "Closing gripper to grasp sensor");
  // gripper_group.setNamedTarget("close");
  // gripper_group.move();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<std::string> object_ids = {
      "Sensor"}; // You need to specify the object ID
  std::map<std::string, moveit_msgs::msg::CollisionObject> objects =
      planning_scene_interface.getObjects(object_ids);

  if (objects.find("Sensor") != objects.end()) {
    // Remove the object (Sensor) temporarily from the planning scene
    planning_scene_interface.removeCollisionObjects({"Sensor"});

    // Perform the gripper motion (close the gripper)
    gripper_group.setNamedTarget("close");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (gripper_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      gripper_group.execute(plan);
    } else {
      RCLCPP_ERROR(logger, "Failed to plan gripper movement.");
    }

    // Add the sensor object back to the planning scene
    planning_scene_interface.addCollisionObjects({objects["Sensor"]});

    RCLCPP_INFO(logger, "Attacting Sensor to Gripper");
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = "UR5_tool0";
    attached_object.object = objects["Sensor"];

    // Define gripper links that can touch the object (adjust for your gripper)
    attached_object.touch_links = {"UR5_robotiq_85_left_finger_tip_link",
                                   "UR5_robotiq_85_right_finger_tip_link"};

    // Apply the attached object to planning scene
    planning_scene_interface.applyAttachedCollisionObject(attached_object);

  } else {
    RCLCPP_WARN(logger, "Sensor object not found in planning scene.");
  }

  // Wait a moment for the grasp to complete
  rclcpp::sleep_for(std::chrono::seconds(2));

  // Move linearly in z-axis to post-grasp position
  RCLCPP_INFO(logger,
              "Moving linearly to post-grasp position (7cm up in z-axis)");

  // Clear previous waypoints
  waypoints.clear();

  // Get current pose after grasping
  current_pose = move_group_interface.getCurrentPose(end_effector_link);
  start_pose = current_pose.pose;

  // Set the target pose for lifting (7cm in z direction)
  geometry_msgs::msg::Pose lift_waypoint = start_pose;
  lift_waypoint.position.z += 0.08; // 7cm up

  waypoints.push_back(lift_waypoint);

  // Try to compute cartesian path for lifting
  fraction = move_group_interface.computeCartesianPath(waypoints, eef_step,
                                                       trajectory);

  if (fraction > 0.0) {
    // Execute the lifting path
    plan.trajectory = trajectory;

    RCLCPP_INFO(logger, "Visualizing Moving Up trajectory");
    visual_tools.publishAxisLabeled(lift_waypoint, "pre-grasp-position");
    visual_tools.publishText(text_pose, "Linear_Up", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory, joint_model_group);
    visual_tools.trigger();

    move_group_interface.execute(plan);
    RCLCPP_INFO(logger, "Linear lifting executed (%.1f%% of path achieved)",
                fraction * 100.0);
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

  } else {
    RCLCPP_ERROR(logger, "Failed to compute cartesian path for lifting. "
                         "Falling back to regular planning.");
    move_group_interface.setPoseTarget(post_grasp_pose);
    success = (move_group_interface.plan(plan) ==
               moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      move_group_interface.execute(plan); // plan = move_group_interface.plan()
      success = (move_group_interface.plan(plan) ==
                 moveit::core::MoveItErrorCode::SUCCESS);
    } else {
      RCLCPP_ERROR(logger, "Planning to post-grasp position failed!");
      return 1;
    }
    move_group_interface.clearPathConstraints();
  }

  RCLCPP_INFO(logger, "Successfully picked up the sensor object!");

  rclcpp::sleep_for(std::chrono::seconds(1));
  // Move Back to Home position
  RCLCPP_INFO(logger, "Moving to Home position");
  // move_group_interface.setNamedTarget("Home");

  geometry_msgs::msg::Pose home_pose;
  home_pose.position.x = -0.109;
  home_pose.position.y = 0.039;
  home_pose.position.z = 0.616;

  home_pose.orientation.x = 0.9553;
  home_pose.orientation.y = 0.0026;
  home_pose.orientation.z = 0.0019;
  home_pose.orientation.w = -0.2957;

  moveit::core::RobotStatePtr current_state =
      move_group_interface.getCurrentState(
          5.0); // Wait up to 5s to get the current state#include
                // "rcutils/logging_macros.h"
  if (current_state) {
    // move_group_interface.setStartState(*current_state);
    RCLCPP_INFO(logger, "Start state updated from current robot state");
  } else {
    RCLCPP_WARN(logger, "Failed to get current robot state");
  }
  move_group_interface.clearPathConstraints();
  
  moveit_msgs::msg::Constraints constraints_home;
  
  rclcpp::sleep_for(std::chrono::seconds(1));
  wrist_1_constraint.joint_name = "UR5_wrist_1_joint"; // Name of the joint
  wrist_1_constraint.position = 0.0; // Center position (0 radians)
  wrist_1_constraint.tolerance_above = M_PI/2 ; // ±180 degrees tolerance
  wrist_1_constraint.tolerance_below = M_PI/2 ; // ±180 degrees tolerance
  wrist_1_constraint.weight = 0.8; // Explicitly setting weight to 1.0 for importance
  
  wrist_2_constraint.joint_name = "UR5_wrist_2_joint"; // Name of the joint
  wrist_2_constraint.position = M_PI/2;         // Center position (0 radians)
  wrist_2_constraint.tolerance_above = M_PI/2 ; // ±180 degrees tolerance
  wrist_2_constraint.tolerance_below = M_PI/2 ; // ±180 degrees tolerance
  wrist_2_constraint.weight = 0.8; // Explicitly setting weight to 1.0 for importance

  wrist_3_constraint.joint_name = "UR5_wrist_3_joint"; // Name of the joint
  wrist_3_constraint.position = M_PI;         // Center position (0 radians)
  wrist_3_constraint.tolerance_above = M_PI/2 ; // ±180 degrees tolerance
  wrist_3_constraint.tolerance_below = M_PI/2 ; // ±180 degrees tolerance
  wrist_3_constraint.weight = 0.8; // Explicitly setting weight to 1.0 for importance
  
  constraints_home.joint_constraints.push_back(wrist_1_constraint);
  constraints_home.joint_constraints.push_back(wrist_2_constraint);
  constraints_home.joint_constraints.push_back(wrist_3_constraint);
  move_group_interface.setPathConstraints(constraints_home);
  
  rclcpp::sleep_for(std::chrono::seconds(1));
  
  
  move_group_interface.setNamedTarget("Home");

  visual_tools.publishText(text_pose, "Calculating_Path_Home", rvt::WHITE,
                           rvt::XLARGE);
  visual_tools.trigger();

  rclcpp::sleep_for(std::chrono::seconds(1));
  //move_group_interface.clearPathConstraints();


  moveit::planning_interface::MoveGroupInterface::Plan plan_home;
  success = (move_group_interface.plan(plan_home) ==
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
  // move_group_interface.move();

  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  // move_group_interface.detachObject("Sensor");


  // Create and publish the completion message
  std_msgs::msg::String msg;
  msg.data = "Sensor_Pickup_Complete";
  cloud_message_publisher->publish(msg);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
