from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Declare a launch argument for the namespace
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='UR3', description='Namespace for the robot'
    )

    # Use the MoveItConfigsBuilder to get the configuration for the robot
    moveit_config = MoveItConfigsBuilder("ur3", package_name="ur3_camera_moveit_config").to_moveit_configs()

    # Generate the Move Group launch
    move_group_launch = generate_move_group_launch(moveit_config)

    # Add the PushRosNamespace action to ensure that everything runs under the given namespace
    namespace_push = PushRosNamespace('UR3')  # Replace 'robot' with the dynamic argument if needed

    # The launch description will include the namespace argument, namespace push, and the move group launch
    return LaunchDescription([
        namespace_arg,
        namespace_push,
        move_group_launch
    ])

