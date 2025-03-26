import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/buhrmann/ws_moveit/install/ros2_azure_bridge'
