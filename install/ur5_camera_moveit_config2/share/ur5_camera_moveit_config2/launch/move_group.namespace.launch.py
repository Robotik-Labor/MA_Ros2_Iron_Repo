from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    # Use the MoveItConfigsBuilder to get the configuration for the robot
    moveit_config = MoveItConfigsBuilder("ur5", package_name="ur5_camera_moveit_config").to_moveit_configs()

    # Generate the Move Group launch
    move_group_launch = generate_move_group_launch(moveit_config)

    # Add the PushRosNamespace action to ensure that everything runs under the given namespace
    namespace_push = PushRosNamespace('UR5')  # Replace 'robot' with the dynamic argument if needed

    # The launch description will include the namespace argument, namespace push, and the move group launch
    return LaunchDescription([
        namespace_push,
        move_group_launch
    ])

