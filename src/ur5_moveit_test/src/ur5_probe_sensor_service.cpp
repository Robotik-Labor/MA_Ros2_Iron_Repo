#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>   // For trigonometrical functions
#include <tf2/LinearMath/Quaternion.h>
#include <common_services_package/srv/get_plantpot_coords.hpp>   // Import service message - adjust to your actual service type
#include <moveit_visual_tools/moveit_visual_tools.h>

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

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(
      node, "world", "/UR5/move_group_tutorial",
      move_group_interface.getRobotModel());
  visual_tools.deleteAllMarkers();

  // RViz provides many types of markers, in this demo we will use text,
  // cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Probing_Plant", rvt::WHITE,
                           rvt::XLARGE);
  visual_tools.trigger();

  const moveit::core::RobotModelConstPtr& robot_model = move_group_interface.getRobotModel();
  const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("ur5_arm");

  

  // Create the MoveGroup interface for the gripper
  moveit::planning_interface::MoveGroupInterface::Options gripper_options(
      "gripper", "robot_description", "/UR5");
  auto gripper_group = MoveGroupInterface(node, gripper_options);

  // Create a publisher for visualization markers
  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
    "UR5/probe_points", rclcpp::QoS(16));

  // Vector to store target poses
  std::vector<geometry_msgs::msg::Pose> target_poses;
  
  // Create a service client to get the last point
  auto client = node->create_client<common_services_package::srv::GetPlantpotCoords>("/UR5/service/plantpot_coords");

  // Wait for the service to be available
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger, "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(logger, "Waiting for service to appear...");
  }

  // Create a request
  auto request = std::make_shared<common_services_package::srv::GetPlantpotCoords::Request>();
  
  // Send the request
  auto result_future = client->async_send_request(request);
  
  // Wait for the result
  if (rclcpp::spin_until_future_complete(node, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto msg = result_future.get()->coordinates;
    RCLCPP_INFO(logger, "Received point: x=%.3f, y=%.3f, z=%.3f", msg.x, msg.y, msg.z);
    
    // Center of the semicircle
    double center_x = msg.x;
    double center_y = msg.y;
    double center_z = msg.z + 0.17;

    // Define the radius of the semicircle
    const double radius = 0.24;
    const int num_positions = 8;
    const double angle_step = M_PI / (num_positions - 1);  // Divide the semicircle into equal parts

    //calculation Point nearest (0,0,z)
    double start_angle = atan2(center_y, center_x) + M_PI;

    double Angles[8] = {start_angle , start_angle + M_PI * 3/28, start_angle +  M_PI * 6/28, start_angle + M_PI * 9/28, start_angle +  M_PI * 12/28, start_angle +  M_PI * 15/28 , start_angle +  M_PI * 18/28,  start_angle + M_PI * 3/4};
    
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

      // Step 3: Define the reference vector (assuming robot arm points along Z-axis)
      double rx = 0, ry = 0, rz = 1; // World Z-axis

      // Step 4: Compute the cross product (axis of rotation)
      double axis_x = ry * vz/Norm  - rz * vy/Norm;
      double axis_y = rz * vx/Norm - rx * vz/Norm;
      double axis_z = rx * vy/Norm - ry * vx/Norm;

      double dotProduct = rx * vx + ry * vy + rz * vz;
      double angle = acos(dotProduct); // Angle between the two vectors

      // Normalize the rotation axis
      double norm = sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z);
      axis_x /= norm;
      axis_y /= norm;
      axis_z /= norm;

      target_pose.orientation.x = axis_x * sin(angle / 2);
      target_pose.orientation.y = axis_y * sin(angle / 2);  // Y-axis aligned with world -Z-axis
      target_pose.orientation.z = axis_z * sin(angle / 2);
      target_pose.orientation.w = cos(angle / 2);  // Identity quaternion

      tf2::Quaternion arm_orientation;
      arm_orientation.setX(target_pose.orientation.x);
      arm_orientation.setY(target_pose.orientation.y);
      arm_orientation.setZ(target_pose.orientation.z);
      arm_orientation.setW(target_pose.orientation.w);

      double angle_to_rotate_z = atan2(vy, vx);

      // Define the fixed camera rotation quaternion
      tf2::Quaternion camera_rotation;
      camera_rotation.setRPY(0.0, 0.0, angle_to_rotate_z - M_PI/2);

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

      // Create a marker to visualize the target position in RViz
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = rclcpp::Clock().now();
      marker.ns = "target_positions";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = target_pose.position;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = 1.0;

      // Publish the marker
      marker_pub->publish(marker);
      
      geometry_msgs::msg::Pose above_target_pose;
      above_target_pose = target_pose;
      above_target_pose.position.z = center_z + 0.07;
      
      // Create a marker to visualize the target position in RViz
      visualization_msgs::msg::Marker marker_top;
      marker_top.header.frame_id = "world";
      marker_top.header.stamp = rclcpp::Clock().now();
      marker_top.ns = "target_positions";
      marker_top.id = i + num_positions;
      marker_top.type = visualization_msgs::msg::Marker::SPHERE;
      marker_top.action = visualization_msgs::msg::Marker::ADD;
      marker_top.pose.position = above_target_pose.position;
      marker_top.pose.orientation.w = 1.0;
      marker_top.scale.x = 0.05;
      marker_top.scale.y = 0.05;
      marker_top.scale.z = 0.05;
      marker_top.color.r = r;
      marker_top.color.g = g;
      marker_top.color.b = b;
      marker_top.color.a = 1.0;
      
      // Publish the marker
      marker_pub->publish(marker_top);
      
      target_poses.push_back(above_target_pose);
    }

    RCLCPP_INFO(logger, "Positions calculated, moving to target positions...");
  } else {
    RCLCPP_ERROR(logger, "Failed to call service");
    return 1;
  }
  
  
  moveit_msgs::msg::JointConstraint shoulder_pan_constraint;
  shoulder_pan_constraint.joint_name = "UR5_shoulder_pan_joint"; // Name of the joint
  shoulder_pan_constraint.position = M_PI / 2; // Center position (0 radians)
  shoulder_pan_constraint.tolerance_above = M_PI/2 ; // ±180 degrees tolerance
  shoulder_pan_constraint.tolerance_below = M_PI/2 ; // ±180 degrees tolerance
  shoulder_pan_constraint.weight = 0.8; // Explicitly setting weight to 1.0 for importance

  // Create joint constraints for the wrist joints
  moveit_msgs::msg::JointConstraint wrist_1_constraint;
  wrist_1_constraint.joint_name = "UR5_wrist_1_joint"; // Name of the joint
  wrist_1_constraint.position = 0.0; // Center position (0 radians)
  wrist_1_constraint.tolerance_above = M_PI; // ±180 degrees tolerance
  wrist_1_constraint.tolerance_below = M_PI; // ±180 degrees tolerance
  wrist_1_constraint.weight = 0.8; // Explicitly setting weight to 1.0 for importance

  moveit_msgs::msg::JointConstraint wrist_2_constraint;
  wrist_2_constraint.joint_name = "UR5_wrist_2_joint"; // Name of the joint
  wrist_2_constraint.position = M_PI/2;         // Center position (0 radians)
  wrist_2_constraint.tolerance_above = M_PI; // ±180 degrees tolerance
  wrist_2_constraint.tolerance_below = M_PI; // ±180 degrees tolerance
  wrist_2_constraint.weight = 0.8; // Explicitly setting weight to 1.0 for importance

  moveit_msgs::msg::JointConstraint wrist_3_constraint;
  wrist_3_constraint.joint_name = "UR5_wrist_3_joint"; // Name of the joint
  wrist_3_constraint.position = M_PI;         // Center position (0 radians)
  wrist_3_constraint.tolerance_above = M_PI; // ±180 degrees tolerance
  wrist_3_constraint.tolerance_below = M_PI; // ±180 degrees tolerance
  wrist_3_constraint.weight = 0.8; // Explicitly setting weight to 1.0 for importance

  moveit_msgs::msg::Constraints constraints;
  //constraints.joint_constraints.push_back(shoulder_pan_constraint);
  constraints.joint_constraints.push_back(wrist_1_constraint);
  constraints.joint_constraints.push_back(wrist_2_constraint);
  constraints.joint_constraints.push_back(wrist_3_constraint);
  move_group_interface.setPathConstraints(constraints);

  
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
      RCLCPP_INFO(logger, "Motion_planning_start");

      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, "Checking_Path_to_Pose_" + std::to_string(i + 1) , rvt::WHITE, rvt::XLARGE);
      visual_tools.trigger();
      
      move_group_interface.setPoseTarget(target_poses[i]);

      success = (move_group_interface.plan(plan_to_above) == moveit::core::MoveItErrorCode::SUCCESS);
      if (!success){ continue;}
      visual_tools.deleteAllMarkers();
      visual_tools.trigger();
      RCLCPP_INFO(logger, "Visualizing Probe trajectory line");
      visual_tools.publishText(text_pose, "Moving_to_Probe_point_" + std::to_string(i + 1), rvt::WHITE, rvt::XLARGE);
      visual_tools.publishTrajectoryLine(plan_to_above.trajectory, joint_model_group);
      visual_tools.trigger();

      move_group_interface.execute(plan_to_above);
      waypoints.clear();

      // Set the target pose for probing (7cm down in z direction)
      probe_point = target_poses[i];
      probe_point.position.z -= 0.07; // 7cm down

      waypoints.push_back(probe_point);
    
      fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, trajectory_down);
      RCLCPP_INFO(logger, "Fraction: %.2f", fraction);
      if (fraction > 0.8) {
        RCLCPP_INFO(logger, "Motion_planning_successful...");
        plan_linar_down.trajectory = trajectory_down;
        visual_tools.deleteAllMarkers();
        visual_tools.trigger();
        RCLCPP_INFO(logger, "Visualizing Home trajectory line");
        visual_tools.publishText(text_pose, "Moving_Down", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(plan_linar_down.trajectory, joint_model_group);
        visual_tools.trigger();


        move_group_interface.execute(plan_linar_down);
        break;
      }
      else
      {
        move_group_interface.setNamedTarget("Home");
        success = (move_group_interface.plan(plan_to_home) == moveit::core::MoveItErrorCode::SUCCESS);
        visual_tools.deleteAllMarkers();
        visual_tools.trigger();
        RCLCPP_INFO(logger, "Visualizing Home trajectory line");
        visual_tools.publishText(text_pose, "Failed_Moving_Failed", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(plan_to_home.trajectory, joint_model_group);
        visual_tools.trigger();
        move_group_interface.execute(plan_to_home);
      }
  }
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  //move_group_interface.setNamedTarget("Home");
  //success = (move_group_interface.plan(plan_to_home) == moveit::core::MoveItErrorCode::SUCCESS);
  //move_group_interface.execute(plan_to_home);
  
  // Shutdown ROS after the task is finished
  rclcpp::shutdown();
  return 1;
}
