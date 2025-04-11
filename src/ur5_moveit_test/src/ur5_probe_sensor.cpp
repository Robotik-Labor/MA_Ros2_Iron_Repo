#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>   // For trigonometrical functions
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
      "sensor_probe_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("sensor_probe_node");
  
  // Create the MoveGroup interface for the UR5 arm
  using moveit::planning_interface::MoveGroupInterface;
  moveit::planning_interface::MoveGroupInterface::Options options(
      "ur5_arm", "robot_description", "/UR5");
  auto move_group_interface = MoveGroupInterface(node, options);
  move_group_interface.setPlanningTime(20.0);
  move_group_interface.setMaxVelocityScalingFactor(0.1);
  move_group_interface.setMaxAccelerationScalingFactor(0.1);
  move_group_interface.setPlannerId("BiTRRT");
  move_group_interface.setNumPlanningAttempts(20);
  move_group_interface.setGoalJointTolerance(0.03);

  // Create the MoveGroup interface for the gripper
  moveit::planning_interface::MoveGroupInterface::Options gripper_options(
#include <cmath>
      "gripper", "robot_description", "/UR5");
  auto gripper_group = MoveGroupInterface(node, gripper_options);


  // Create a publisher for visualization markers
  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
    "UR5/probe_points", rclcpp::QoS(10));

  std::vector<geometry_msgs::msg::Pose> target_poses;
  // Subscriber to /UR5/AboveCup topic to calculate positions
  auto subscription = node->create_subscription<geometry_msgs::msg::Point>(
    "/UR5/plantpot_coords", 10, [&move_group_interface, &logger, &target_poses, &marker_pub](const geometry_msgs::msg::Point::SharedPtr msg) {
      RCLCPP_INFO(logger, "Received point: x=%.2f, y=%.2f, z=%.2f", msg->x, msg->y, msg->z);
      
      // Center of the semicircle
      double center_x = msg->x;
      double center_y = msg->y;
      double center_z = msg->z +0.17;

      // Define the radius of the semicircle
      const double radius = 0.24;
      const int num_positions = 5;
      const double angle_step = M_PI / (num_positions - 1);  // Divide the semicircle into equal parts

      //calulation Point nearest (0,0,z)
      double start_angle = atan2(center_y,center_x) + M_PI ;

      double Angles[5] = {start_angle - M_PI/2 ,start_angle - M_PI/4, start_angle, start_angle + M_PI/4 ,start_angle + M_PI/2};
      
      const float colors[10][3] = {
	    {1.0f, 0.0f, 0.0f},     // Red
	    {0.0f, 1.0f, 0.0f},     // Green
	    {0.0f, 0.0f, 1.0f},     // Blue
	    {1.0f, 1.0f, 0.0f},     // Yellow
	    {1.0f, 0.0f, 1.0f},     // Magenta
	    {0.0f, 1.0f, 1.0f},     // Cyan
	    {0.5f, 0.0f, 0.0f},     // Dark Red
	    {0.0f, 0.5f, 0.0f},     // Dark Green
	    {0.0f, 0.0f, 0.5f},     // Dark Blue
	    {1.0f, 0.5f, 0.0f}      // Orange
	};

      

      // Calculate the 5 positions
      for (int i = 0; i < num_positions; ++i) {
        //double angle_for_position = i * angle_step;  // Angle for current position
        double x = center_x + radius * cos(Angles[i]);
        double y = center_y + radius * sin(Angles[i]);

        // Define target pose (Y-axis aligned with the world negative Z-axis)
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = center_z;

        double vx = center_x - x;
        double vy = center_y - y;
        double vz = center_z;

        
        
        double Norm = sqrt(vx*vx + vy*vy + vz*vz);
        // Calculate the angle

        // Step 3: Define the reference vector (assuming robot arm points along Z-axis)
        double rx = 0, ry = 0, rz = 1; // World Z-axis

        // Step 4: Compute the cross product (axis of rotation)
        double axis_x = ry * vz/Norm  - rz * vy/Norm;
        double axis_y = rz * vx/Norm - rx * vz/Norm ;
        double axis_z = rx * vy/Norm - ry * vx/Norm;

        double dotProduct = rx * vx + ry * vy + rz * vz;
        double angle = acos(dotProduct); // Angle between the two vectors
        //angle = 2.21430;
        // Normalize the rotation axis
        double norm = sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z);
        axis_x /= norm;
        axis_y /= norm;
        axis_z /= norm;

        target_pose.orientation.x = axis_x * sin(angle / 2);
        target_pose.orientation.y = axis_y * sin(angle / 2);  // Y-axis aligned with world -Z-axis
        target_pose.orientation.z = axis_z * sin(angle / 2);
        target_pose.orientation.w = cos(angle / 2);  // Identity quaternion

        // After your existing code that calculates the robot arm's orientation (quaternion for target_pose)

        tf2::Quaternion arm_orientation;
        arm_orientation.setX(target_pose.orientation.x);
        arm_orientation.setY(target_pose.orientation.y);
        arm_orientation.setZ(target_pose.orientation.z);
        arm_orientation.setW(target_pose.orientation.w);

        double angle_to_rotate_z = atan2(vy, vx);

        // Define the fixed camera rotation quaternion (e.g., rotating 180 degrees around Y-axis)
        tf2::Quaternion camera_rotation;
        camera_rotation.setRPY(0.0, 0.0, angle_to_rotate_z - M_PI/2 );  // Rotation by 180 degrees around Z-axis

        // Combine the two quaternions
        tf2::Quaternion final_orientation = arm_orientation * camera_rotation;

        // Set the final orientation to the target pose
        target_pose.orientation.x = final_orientation.x();
        target_pose.orientation.y = final_orientation.y();
        target_pose.orientation.z = final_orientation.z();
        target_pose.orientation.w = final_orientation.w();
        
       	


	float r = colors[i % 10][0];
	float g = colors[i % 10][1];
	float b = colors[i % 10][2];


        // Add the pose to the target_poses list


        // Create a marker to visualize the target position in RViz
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";  // Use "world" or the correct frame
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "target_positions";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = target_pose.position;
        marker.pose.orientation.w = 1.0;  // No rotation for the marker
        marker.scale.x = 0.05;  // Marker size (radius)
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = r;  // Red color
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;  // Fully opaque

        // Publish the marker
        marker_pub->publish(marker);
        
        
        
	geometry_msgs::msg::Pose above_target_pose;
	above_target_pose = target_pose;
	above_target_pose.position.z = center_z + 0.07;
#include <cmath>
	
	// Create a marker to visualize the target position in RViz
        visualization_msgs::msg::Marker marker_top;
        marker_top.header.frame_id = "world";  // Use "world" or the correct frame
        marker_top.header.stamp = rclcpp::Clock().now();
        marker_top.ns = "target_positions";
        marker_top.id = i + num_positions;
        marker_top.type = visualization_msgs::msg::Marker::SPHERE;
        marker_top.action = visualization_msgs::msg::Marker::ADD;
        marker_top.pose.position = above_target_pose.position;
        marker_top.pose.orientation.w = 1.0;  // No rotation for the marker
        marker_top.scale.x = 0.05;  // Marker size (radius)
        marker_top.scale.y = 0.05;
        marker_top.scale.z = 0.05;
        marker_top.color.r = r;  // Red color
        marker_top.color.g = g;
        marker_top.color.b = b;
        marker_top.color.a = 1.0;  // Fully opaque
	
	// Publish the marker
        marker_pub->publish(marker_top);
        
        target_poses.push_back(above_target_pose);
        //target_poses.push_back(target_pose);
      }

      RCLCPP_INFO(logger, "Positions calculated, moving to target positions...");
    });

  // Ensure the program waits until the positions are calculated
  while (target_poses.empty()) {
    rclcpp::spin_some(node);  // Wait for the subscription callback to fill target_poses
  }
  
  // Create joint constraints for the wrist joints
  moveit_msgs::msg::JointConstraint wrist_1_constraint;
  wrist_1_constraint.joint_name = "UR5_wrist_1_joint"; // Name of the joint
  wrist_1_constraint.position = -M_PI / 2; // Center position (0 radians)
  wrist_1_constraint.tolerance_above = M_PI / 1.5; // ±180 degrees tolerance
  wrist_1_constraint.tolerance_below = M_PI / 1.5; // ±180 degrees tolerance
  wrist_1_constraint.weight = 0.8; // Explicitly setting weight to 1.0 for importance

  moveit_msgs::msg::JointConstraint wrist_2_constraint;
  wrist_2_constraint.joint_name = "UR5_wrist_2_joint"; // Name of the joint
  wrist_2_constraint.position = 0.0;         // Center position (0 radians)
  wrist_2_constraint.tolerance_above = M_PI/2; // ±180 degrees tolerance
  wrist_2_constraint.tolerance_below = M_PI/2; // ±180 degrees tolerance
  wrist_2_constraint.weight =
      0.8; // Explicitly setting weight to 1.0 for importance

  moveit_msgs::msg::JointConstraint wrist_3_constraint;
  wrist_3_constraint.joint_name = "UR5_wrist_3_joint"; // Name of the joint
  wrist_3_constraint.position = 0.0;         // Center position (0 radians)
  wrist_3_constraint.tolerance_above = M_PI/2; // ±180 degrees tolerance
  wrist_3_constraint.tolerance_below = M_PI/2; // ±180 degrees tolerance
  wrist_3_constraint.weight =
      0.8; // Explicitly setting weight to 1.0 for importance

  moveit_msgs::msg::Constraints constraints;
  // constraints.joint_constraints.push_back(shoulder_lift_constraint);
  constraints.joint_constraints.push_back(wrist_1_constraint);
  constraints.joint_constraints.push_back(wrist_2_constraint);
  constraints.joint_constraints.push_back(wrist_3_constraint);
  //move_group_interface.setPathConstraints(constraints);
  
  bool success;
  moveit::planning_interface::MoveGroupInterface::Plan plan_to_above;
  moveit::planning_interface::MoveGroupInterface::Plan plan_linar_down;
  moveit_msgs::msg::RobotTrajectory trajectory_down;
  geometry_msgs::msg::Pose probe_point;
  const double eef_step = 0.01;
  double fraction;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose start_pose;
  std::vector<double> target_joint_degrees = {90.0, -156.0, 103.0, 80.0, 91.0, 0.0};  // Example joint values in degrees
  std::vector<double> radians;
  for (double degree : target_joint_degrees) {
        radians.push_back(degree * M_PI / 180.0);  // Convert to radians
    }
  moveit::planning_interface::MoveGroupInterface::Plan plan_to_home;
  
  for (int i = 0; i < target_poses.size(); i++) {
  	  RCLCPP_INFO(logger, "Motion_planning_staret");
  	  move_group_interface.setPoseTarget(target_poses[i]);

	  success = (move_group_interface.plan(plan_to_above) == moveit::core::MoveItErrorCode::SUCCESS);
	  if (!success){ continue;}
	  move_group_interface.execute(plan_to_above);
	  waypoints.clear();
	  // Get current pose before grasping
	  //current_pose = move_group_interface.getCurrentPose(end_effector_link);

	  // Set the target pose for lifting (7cm in z direction)
	  probe_point = target_poses[i];
	  probe_point.position.z -= 0.07; // 7cm up

	  waypoints.push_back(probe_point);
	
	  fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, trajectory_down);
	  RCLCPP_INFO(logger, "Fraction: %.2f", fraction);
	  if (fraction > 0.8) {
	  RCLCPP_INFO(logger, "Motion_planning_successfull...");
	  plan_linar_down.trajectory = trajectory_down;
	  move_group_interface.execute(plan_linar_down);  
	  
	  break;
	  }
	  else
	  {
	  move_group_interface.setNamedTarget("Home");
	  success = (move_group_interface.plan(plan_to_home) == moveit::core::MoveItErrorCode::SUCCESS);
	  move_group_interface.execute(plan_to_home);
	  }
	  
	  		
  }
  
  move_group_interface.setNamedTarget("Home");
  success = (move_group_interface.plan(plan_to_home) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group_interface.execute(plan_to_home);
  // Shutdown ROS after the task is finished
  rclcpp::shutdown();
  return 0;
}
