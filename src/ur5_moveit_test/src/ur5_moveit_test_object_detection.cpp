#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>   // For trigonometrical functions
#include <tf2/LinearMath/Quaternion.h>

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

  // Create a publisher for visualization markers
  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
    "visualization_marker", rclcpp::QoS(10));

  // Subscriber to /UR5/AboveCup topic to calculate positions
  auto subscription = node->create_subscription<geometry_msgs::msg::Point>(
    "/UR5/AboveCup", 10, [&move_group_interface, &logger, &target_poses, &marker_pub](const geometry_msgs::msg::Point::SharedPtr msg) {
      RCLCPP_INFO(logger, "Received point: x=%.2f, y=%.2f, z=%.2f", msg->x, msg->y, msg->z);
      
      // Center of the semicircle
      double center_x = msg->x;
      double center_y = msg->y;
      double center_z = msg->z;

      // Define the radius of the semicircle
      const double radius = 0.2;
      const int num_positions = 5;
      const double angle_step = M_PI / (num_positions - 1);  // Divide the semicircle into equal parts

      //calulation Point nearest (0,0,z)
      double start_angle = atan2(center_y,center_x) + M_PI ;

      double Angles[5] = {start_angle - M_PI/2 ,start_angle - M_PI/4, start_angle, start_angle + M_PI/4 ,start_angle + M_PI/2};
      

      // Calculate the 5 positions
      for (int i = 0; i < num_positions; ++i) {
        double angle_for_position = i * angle_step;  // Angle for current position
        double x = center_x + radius * cos(Angles[i]);
        double y = center_y + radius * sin(Angles[i]);

        // Define target pose (Y-axis aligned with the world negative Z-axis)
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = center_z;

        double vx = center_x - x;
        double vy = center_y - y;
        double vz = -0.8;

        
        
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


        // Add the pose to the target_poses list
        target_poses.push_back(target_pose);


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
        marker.color.r = 1.0;  // Red color
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;  // Fully opaque

        // Publish the marker
        marker_pub->publish(marker);
      }

      RCLCPP_INFO(logger, "Positions calculated, moving to target positions...");
    });

  // Ensure the program waits until the positions are calculated
  while (target_poses.empty()) {
    rclcpp::spin_some(node);  // Wait for the subscription callback to fill target_poses
  }
    
  moveit_msgs::msg::JointConstraint shoulder_lift_constraint;
  shoulder_lift_constraint.joint_name = "UR5_shoulder_lift_joint";  // Name of the joint
  shoulder_lift_constraint.position = -M_PI/2;  // Desired joint position
  shoulder_lift_constraint.tolerance_above = M_PI/2;  // Tolerance above the desired position
  shoulder_lift_constraint.tolerance_below = M_PI/2;  // Tolerance below the desired position
  shoulder_lift_constraint.weight = 1.0;  // Explicitly setting weight to 1.0 for importance

  // Create joint constraints for the wrist joints
  moveit_msgs::msg::JointConstraint wrist_1_constraint;
  wrist_1_constraint.joint_name = "UR5_wrist_1_joint";  // Name of the joint
  wrist_1_constraint.position = 0.0;  // Center position (0 radians)
  wrist_1_constraint.tolerance_above = M_PI;  // ±180 degrees tolerance
  wrist_1_constraint.tolerance_below = M_PI;  // ±180 degrees tolerance
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
  
  // Move the robot to each position
  for (const auto& target_pose : target_poses) {
    bool success = false;
    int attempts = 0;

    // Retry mechanism: attempt up to 3 times
    while (attempts < 3 && !success) {
      move_group_interface.setPoseTarget(target_pose);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      move_group_interface.setPathConstraints(constraints);
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
  std::vector<double> joint_values = {
      -0.78539,   // UR5_shoulder_pan_joint
      -2.5122,   // UR5_shoulder_lift_joint
      1.9107,    // UR5_elbow_joint
      -1.8568,   // UR5_wrist_1_joint
      -1.6435,   // UR5_wrist_2_joint
      0.0347     // UR5_wrist_3_joint
  };

  // Set the joint values as the target positions
  move_group_interface.setJointValueTarget(joint_values);
  moveit::planning_interface::MoveGroupInterface::Plan return_plan;
  //move_group_interface.setPathConstraints(constraints);
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
