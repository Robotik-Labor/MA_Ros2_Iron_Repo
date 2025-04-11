from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_azure_bridge')
    
    
    
    # Create the node
    bridge_node = Node(
        package='ros2_azure_bridge',
        executable='service_calls',
        name='service_calls',
        output='screen'
    )
    
    return LaunchDescription([
        bridge_node
    ])
