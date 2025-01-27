import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='UR5', description='Namespace for the node'),
        Node(
            package='your_package_name',
            executable='gripper_close_moveit_test',
            name='ur5_move_test',
            namespace=[LaunchConfiguration('namespace')],  # Namespace from launch argument
            output='screen'
        ),
    ])

