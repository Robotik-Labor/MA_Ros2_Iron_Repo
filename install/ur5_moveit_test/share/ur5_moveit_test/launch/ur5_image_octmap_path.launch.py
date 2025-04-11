import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur5_moveit_test',  # Your package
            executable='ur5_image_octmap_path',  # Your executable name,  # Node name
            namespace='/UR5',  # Set namespace
            output='screen',
        ),
    ])
