#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
//#include "planning_scene_interface.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "sensor_pickup_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("sensor_pickup_node");
  
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });
  // Create the MoveGroup interface for the UR5 arm
  using moveit::planning_interface::MoveGroupInterface;
  moveit::planning_interface::MoveGroupInterface::Options options("ur5_arm", "robot_description", "/UR5");
  auto move_group_interface = MoveGroupInterface(node, options);
  
  // Create the MoveGroup interface for the gripper
  moveit::planning_interface::MoveGroupInterface::Options gripper_options("gripper", "robot_description", "/UR5");
  moveit::planning_interface::MoveGroupInterface gripper_group(node, gripper_options);

  // Allow time for the interfaces to initialize
  rclcpp::sleep_for(std::chrono::seconds(1));

  // Set planning parameters
  move_group_interface.setPlanningTime(15.0);
  move_group_interface.setMaxVelocityScalingFactor(0.1);
  move_group_interface.setMaxAccelerationScalingFactor(0.1);
  move_group_interface.setNumPlanningAttempts(3);
  move_group_interface.setWorkspace(-449, -320, -200, 850, 850, 1200);
  
  moveit_msgs::msg::JointConstraint shoulder_lift_constraint;
  shoulder_lift_constraint.joint_name = "UR5_shoulder_lift_joint";  // Name of the joint
  shoulder_lift_constraint.position = -M_PI/2;  // Desired joint position
  shoulder_lift_constraint.tolerance_above = M_PI/2;  // Tolerance above the desired position
  shoulder_lift_constraint.tolerance_below = M_PI/2;  // Tolerance below the desired position
  shoulder_lift_constraint.weight = 1.0;  // Explicitly setting weight to 1.0 for importance

  // Create joint constraints for the wrist joints
  moveit_msgs::msg::JointConstraint wrist_1_constraint;
  wrist_1_constraint.joint_name = "UR5_wrist_1_joint";  // Name of the joint
  wrist_1_constraint.position = -M_PI/2;  // Center position (0 radians)
  wrist_1_constraint.tolerance_above = M_PI/2;  // ±180 degrees tolerance
  wrist_1_constraint.tolerance_below = M_PI/2;  // ±180 degrees tolerance
  wrist_1_constraint.weight = 0.8;  // Explicitly setting weight to 1.0 for importance

  moveit_msgs::msg::JointConstraint wrist_2_constraint;
  wrist_2_constraint.joint_name = "UR5_wrist_2_joint";  // Name of the joint
  wrist_2_constraint.position = 0.0;  // Center position (0 radians)
  wrist_2_constraint.tolerance_above = M_PI;  // ±180 degrees tolerance
  wrist_2_constraint.tolerance_below = M_PI;  // ±180 degrees tolerance
  wrist_2_constraint.weight = 0.8;  // Explicitly setting weight to 1.0 for importance

  moveit_msgs::msg::JointConstraint wrist_3_constraint;
  wrist_3_constraint.joint_name = "UR5_wrist_3_joint";  // Name of the joint
  wrist_3_constraint.position = 0.0;  // Center position (0 radians)
  wrist_3_constraint.tolerance_above = M_PI;  // ±180 degrees tolerance
  wrist_3_constraint.tolerance_below = M_PI;  // ±180 degrees tolerance
  wrist_3_constraint.weight = 0.8;  // Explicitly setting weight to 1.0 for importance

  moveit_msgs::msg::Constraints constraints;
  constraints.joint_constraints.push_back(shoulder_lift_constraint);
  constraints.joint_constraints.push_back(wrist_1_constraint);
  constraints.joint_constraints.push_back(wrist_2_constraint);
  constraints.joint_constraints.push_back(wrist_3_constraint);
  move_group_interface.setPathConstraints(constraints);
  

  
  // Get and set the reference frame
  std::string reference_frame = move_group_interface.getPlanningFrame();
  RCLCPP_INFO(logger, "Using reference frame: %s", reference_frame.c_str());
  std::string pose_reference_frame = move_group_interface.getPoseReferenceFrame();
  RCLCPP_INFO(logger, "Using reference pose_reference_frame: %s", pose_reference_frame.c_str());
  move_group_interface.setPoseReferenceFrame(pose_reference_frame);
  //RCLCPP_INFO(logger, "Using reference frame: %s", reference_frame.c_str());
  
  // Define the target pose for the sensor object
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.24;
  target_pose.position.y = -0.18;
  target_pose.position.z = 0.11;
  target_pose.orientation.x = 0.51;
  target_pose.orientation.y = -0.49;
  target_pose.orientation.z = 0.51;
  target_pose.orientation.w = -0.49;
  
  // Create a pre-grasp pose with 5cm offset along x-axis
  geometry_msgs::msg::Pose pre_grasp_pose = target_pose;
  pre_grasp_pose.position.z += 0.07;  // 5cm offset along x-axis
  
  // Create a post-grasp pose with 7cm offset along z-axis
  geometry_msgs::msg::Pose post_grasp_pose = target_pose;
  post_grasp_pose.position.z += 0.07;  // 7cm offset along z-axis
  
  // Get the current pose to use as starting point
  geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose("UR5_tool0");
  RCLCPP_INFO(logger, "Current end effector pose: x=%.3f, y=%.3f, z=%.3f", 
              current_pose.pose.position.x, 
              current_pose.pose.position.y, 
              current_pose.pose.position.z);
  
  // Open the gripper
  RCLCPP_INFO(logger, "Opening gripper");
  gripper_group.setNamedTarget("open");
  gripper_group.move();
  
  // Move to pre-grasp position
  RCLCPP_INFO(logger, "Moving to pre-grasp position");
  move_group_interface.setPoseTarget(pre_grasp_pose);
  
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning to pre-grasp position failed!");
    return 1;
  }
  
  // Get the end effector link to use for cartesian path planning
  std::string end_effector_link = move_group_interface.getEndEffectorLink();
  RCLCPP_INFO(logger, "End effector link: %s", end_effector_link.c_str());
  
  // Move linearly from pre-grasp to grasp position
  RCLCPP_INFO(logger, "Moving linearly to grasp position");
  
  // Compute a Cartesian path for linear movement
  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  // Start from the current position (very important for cartesian path planning)
  current_pose = move_group_interface.getCurrentPose(end_effector_link);
RCLCPP_INFO(logger, "getCurrentPose: Position [%.2f, %.2f, %.2f], Orientation [%.2f, %.2f, %.2f, %.2f], Frame: %s",
            current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
            current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w,
            current_pose.header.frame_id.c_str());
  geometry_msgs::msg::Pose start_pose = current_pose.pose;
  

  
  // Try to compute a cartesian path
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.005;  // 1cm resolution
  
  
  // Get current pose before grasping
  current_pose = move_group_interface.getCurrentPose(end_effector_link);
  start_pose = current_pose.pose;
  
  // Set the target pose for lifting (7cm in z direction)
  geometry_msgs::msg::Pose grab_waypoint = start_pose;
  grab_waypoint.position.z -= 0.07;  // 7cm up
  
  waypoints.push_back(grab_waypoint);

  
  double fraction = move_group_interface.computeCartesianPath(
    waypoints, eef_step, trajectory
  );
  
  RCLCPP_INFO(logger, "Cartesian path fraction: %.2f", fraction);
  
  if (fraction > 0.0)
  {
    // Execute the lifting path
    plan.trajectory = trajectory;
    move_group_interface.execute(plan);
    RCLCPP_INFO(logger, "Linear lifting executed (%.1f%% of path achieved)", fraction * 100.0);
  }
  else
  {
    RCLCPP_ERROR(logger, "Failed to compute cartesian path for lifting. Falling back to regular planning.");
    move_group_interface.setPoseTarget(target_pose);
    success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success)
    {
      move_group_interface.execute(plan);
    }
    else
    {
      RCLCPP_ERROR(logger, "Planning to post-grasp position failed!");
      return 1;
    }
  }
  
  // Close the gripper to grasp the sensor
  RCLCPP_INFO(logger, "Closing gripper to grasp sensor");
  //gripper_group.setNamedTarget("close");
  //gripper_group.move();
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<std::string> object_ids = {"Sensor"}; // You need to specify the object ID
  std::map<std::string, moveit_msgs::msg::CollisionObject> objects = planning_scene_interface.getObjects(object_ids);
    
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
	    attached_object.touch_links = {
		"UR5_robotiq_85_left_finger_tip_link",
		"UR5_robotiq_85_right_finger_tip_link"
	    };
       
       // Apply the attached object to planning scene
    	planning_scene_interface.applyAttachedCollisionObject(attached_object);
        
        
    } else {
        RCLCPP_WARN(logger, "Sensor object not found in planning scene.");
    }
    
    
  
  
  
  // Wait a moment for the grasp to complete
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  
  // Move linearly in z-axis to post-grasp position
  RCLCPP_INFO(logger, "Moving linearly to post-grasp position (7cm up in z-axis)");
  
  // Clear previous waypoints
  waypoints.clear();
  
  // Get current pose after grasping
  current_pose = move_group_interface.getCurrentPose(end_effector_link);
  start_pose = current_pose.pose;
  
  // Set the target pose for lifting (7cm in z direction)
  geometry_msgs::msg::Pose lift_waypoint = start_pose;
  lift_waypoint.position.z += 0.07;  // 7cm up
  
  waypoints.push_back(lift_waypoint);
  
  // Try to compute cartesian path for lifting
  fraction = move_group_interface.computeCartesianPath(
    waypoints, eef_step, trajectory
  );
  
  if (fraction > 0.0)
  {
    // Execute the lifting path
    plan.trajectory = trajectory;
    move_group_interface.execute(plan);
    RCLCPP_INFO(logger, "Linear lifting executed (%.1f%% of path achieved)", fraction * 100.0);
  }
  else
  {
    RCLCPP_ERROR(logger, "Failed to compute cartesian path for lifting. Falling back to regular planning.");
    move_group_interface.setPoseTarget(post_grasp_pose);
    success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success)
    {
      move_group_interface.execute(plan);
    }
    else
    {
      RCLCPP_ERROR(logger, "Planning to post-grasp position failed!");
      return 1;
    }
  }
  
  RCLCPP_INFO(logger, "Successfully picked up the sensor object!");
  
  
  // Move Back to Home position
  RCLCPP_INFO(logger, "Moving to Home position");
  move_group_interface.setNamedTarget("Home");
  move_group_interface.move();
  
  move_group_interface.detachObject("Sensor");
  
  
  
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
