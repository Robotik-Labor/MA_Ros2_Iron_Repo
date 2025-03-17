from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch_ros.actions import SetRemap

def generate_launch_description():
    # Use the MoveItConfigsBuilder to get the configuration for the robot
    moveit_config = MoveItConfigsBuilder("ur5", package_name="ur5_camera_moveit_config2").to_moveit_configs()
    
    # Generate the Move Group launch
    move_group_launch = generate_move_group_launch(moveit_config)
    
    # Create a group action for the namespace and remapping
    namespace_actions = GroupAction([
        # Push the namespace
        PushRosNamespace('UR5'),
        
        # Add topic remapping
        #SetRemap( '/UR5/joint_states','/joint_states'),
        
        # Include the move group launch
        move_group_launch
    ])
    
    # Return the launch description
    return LaunchDescription([
        namespace_actions
    ])
