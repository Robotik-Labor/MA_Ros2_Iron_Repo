#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <thread>  // <---- add this to the set of includes at the top

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
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  moveit::planning_interface::MoveGroupInterface::Options options("ur5_arm", "robot_description", "/UR5");
  auto move_group_interface = MoveGroupInterface(node, options); //change

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "UR5_base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()
  };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  auto const prompt = [&moveit_visual_tools](auto text) { moveit_visual_tools.prompt(text); };
  // Set a target Pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.24;
  target_pose.position.y = -0.18;
  target_pose.position.z = 0.11 + 0.07;
  target_pose.orientation.x = 0.51;
  target_pose.orientation.y = -0.49;
  target_pose.orientation.z = 0.51;
  target_pose.orientation.w = -0.49;
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  RCLCPP_INFO(logger, "Test##########################################################################################");
  prompt("Start Planning?");
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success){
    prompt("Planning Success! Start Executing?");
    move_group_interface.execute(plan);
  }
  else
  {
    prompt("Planning Failed!");
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}

