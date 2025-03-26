from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_azure_bridge')
    
    # Default config file
    default_config = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    
    # Launch arguments
    config_file = LaunchConfiguration('config_file')
    
    # Declare launch arguments
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to config file with Azure IoT Hub connection string'
    )
    
    # Create the node
    bridge_node = Node(
        package='ros2_azure_bridge',
        executable='azure_bridge_node',
        name='ros2_azure_iot_bridge',
        parameters=[config_file],
        output='screen'
    )
    
    return LaunchDescription([
        declare_config_file,
        bridge_node
    ])
