#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import threading
import json
import time
import datetime as dt
import ast
import logging
from azure.iot.device import IoTHubDeviceClient, Message, MethodResponse
from std_msgs.msg import String

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ROS2AzureIoTBridge(Node):
    """
    ROS2 Node that bridges communication between Azure IoT Hub and ROS2.
    - Receives commands from IoT Hub and executes appropriate ROS2 launch files
    - Listens for messages on ROS2 topics and forwards them to IoT Hub
    """
    
    def __init__(self):
        super().__init__('ros2_azure_iot_bridge')
        
        # Get connection string parameter or use a default for testing
        self.declare_parameter('connection_string', '')
        self.cloud_config = self.get_parameter('connection_string').get_parameter_value().string_value
        
        if not self.cloud_config:
            self.get_logger().error("No connection string provided. Please set the connection_string parameter.")
            return

        # Initialize IoT Hub client
        self.iot_client = self.create_iot_client()
        
        # Setup ROS2 subscriber for the robot feedback topic
        self.robot_feedback_subscription = self.create_subscription(
            String,
            '/UR5/cloud_messages',
            self.robot_feedback_callback,
            10  # QoS profile
        )
        
        # Store process references to track running scripts
        self.current_process = None
        self.start_time = None
        
        self.get_logger().info("ROS2 Azure IoT Bridge initialized and ready")
        
    def create_iot_client(self) -> IoTHubDeviceClient:
        """Creates an IoTHubDeviceClient client object, connects via the connection string
        
        Returns:
            IoTHubDeviceClient: IoTHub object to handle communication and shutdown
        """
        # Instantiate the client
        client = IoTHubDeviceClient.create_from_connection_string(self.cloud_config)
        try:
            client.connect()
            self.get_logger().info("Connected to Azure IoT Hub")
            
            # Set up the method request handler
            client.on_method_request_received = self.method_request_handler
            
        except Exception as e:
            self.get_logger().error(f"Failed to connect to IoT Hub: {e}")
            self.get_logger().info("Trying again in 5 seconds.")
            time.sleep(5)
            try:
                client.connect()
                self.get_logger().info("Connected to Azure IoT Hub on second attempt")
                # Set up the method request handler
                client.on_method_request_received = self.method_request_handler
            except Exception as e:
                self.get_logger().error(f"Failed to connect again: {e}")
        
        return client
        
    def method_request_handler(self, method_request):
        """Handles incoming requests from the cloud IoT Hub
    
        Args:
            method_request: Request from the cloud IoT Hub with WorkStation and Parameters
        """
        if method_request.name == "StartOrder":
            try:
                # Parse the payload
                payload = method_request.payload
                payloadList = ast.literal_eval(payload)
                self.start_time = dt.datetime.now()
                
                # Create a method response indicating the method request was resolved
                resp_status = 200
                resp_payload = {"Response": "Command received by ROS2 Bridge"}
                method_response = MethodResponse(
                    method_request.request_id, resp_status, resp_payload)
                
                # Send the response
                self.iot_client.send_method_response(method_response)
                
                # Log the received order
                self.get_logger().info(f"StartOrder received for {payloadList['WorkStation']}")
                
                # Process the payload only if it's meant for this device
                if payloadList["WorkStation"] == "ROS_Cobot":
                    payload_content = json.loads(payloadList["Payload"])
                    self.get_logger().info(f"Processing robot command: {payload_content['Funktion']}")
                    
                    # Execute the appropriate ROS2 command based on the function
                    if payload_content["Funktion"] == "Pickup_Sensor":
                        # Run as launch file
                        self.execute_ros2_command("launch", "ur5_moveit_test", "ur5_pickup_sensor.launch.py")
                    elif payload_content["Funktion"] == "Drop_Sensor":
                        # Run as launch file
                        self.execute_ros2_command("launch", "ur5_moveit_test", "ur5_drop_sensor.launch.py")
                    elif payload_content["Funktion"] == "Add_Objects":
                        # Example of running a node directly
                        self.execute_ros2_command("run", "edit_planning_scene", "add_object_to_planning_scene_node")
                    elif payload_content["Funktion"] == "CalibrateCamera":
                        # Example with arguments
                        self.execute_ros2_command("launch", "camera_pkg", "calibrate.launch.py", ["use_sim:=false"])
                    else:
                        # Check if the payload contains explicit command information
                        if "command_type" in payload_content and "package" in payload_content and "executable" in payload_content:
                            self.execute_ros2_command(
                                payload_content["command_type"],
                                payload_content["package"],
                                payload_content["executable"],
                                payload_content.get("arguments", None)
                            )
                        else:
                            self.get_logger().warning(f"Unknown function: {payload_content['Funktion']}")
                else:
                    self.get_logger().info(f"Message not for this device. Targeted workstation: {payloadList['WorkStation']}")
            
            except Exception as e:
                self.get_logger().error(f"Error processing method request: {e}")
                # Send error response
                resp_status = 400
                resp_payload = {"Response": f"Error: {str(e)}"}
                method_response = MethodResponse(
                    method_request.request_id, resp_status, resp_payload)
                # self.iot_client.send_method_response(method_response)
        else:
            # Method not recognized
            resp_status = 404
            resp_payload = {"Response": f"Method '{method_request.name}' not defined"}
            method_response = MethodResponse(
                method_request.request_id, resp_status, resp_payload)
            self.iot_client.send_method_response(method_response)
        
        
        
    
    def execute_ros2_command(self, command_type, package, executable, arguments=None):
        """Execute a ROS2 command in a separate thread
        
        Args:
            command_type: Type of ROS2 command ('launch' or 'run')
            package: ROS2 package name
            executable: Launch file or node executable name
            arguments: Optional list of additional arguments
        """
        # Cancel any already running process
        if self.current_process and self.current_process.poll() is None:
            self.get_logger().warn("Terminating existing process before starting new one")
            self.current_process.terminate()
            self.current_process.wait()
        
        # Start a new thread to execute the command
        thread = threading.Thread(
            target=self._run_ros2_command,
            args=(command_type, package, executable, arguments)
        )
        thread.daemon = True
        thread.start()

    def _run_ros2_command(self, command_type, package, executable, arguments=None):
        """Internal method to run a ROS2 command as a subprocess
        
        Args:
            command_type: Type of ROS2 command ('launch' or 'run')
            package: ROS2 package name
            executable: Launch file or node executable name
            arguments: Optional list of additional arguments
        """
        try:
            # Build the command
            command = ["ros2", command_type, package]
            
            # Add executable
            if command_type == "launch":
                command.append(executable)
            elif command_type == "run":
                command.append(executable)
            
            # Add any additional arguments
            if arguments:
                if isinstance(arguments, list):
                    command.extend(arguments)
                elif isinstance(arguments, str):
                    command.append(arguments)
            
            self.get_logger().info(f"Executing ROS2 {command_type} command: {' '.join(command)}")
            
            # Execute the command
            self.current_process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            # Wait for the process to complete
            stdout, stderr = self.current_process.communicate()
            return_code = self.current_process.returncode
            
            self.get_logger().info(f"Command completed with return code: {return_code}")
            
            # Check for errors
            if return_code != 0:
                self.get_logger().error(f"Error executing command: {stderr}")
                self.send_status_to_iot_hub("Error", f"Command error: {return_code}")
            else:
                self.get_logger().info(f"Command executed successfully\nOutput: {stdout[:200]}...")
                self.send_status_to_iot_hub("Complete", "Command executed successfully")
                    
        except Exception as e:
            self.get_logger().error(f"Exception while executing command: {e}")
            self.send_status_to_iot_hub("Error", f"Exception: {str(e)}")
    
    def robot_feedback_callback(self, msg):
        """Callback function for robot feedback messages
        
        Args:
            msg: Message received on the /UR5/Cloud_MESSAGES topic
        """
        try:
            self.get_logger().info(f"Received robot feedback: {msg.data}")
            
            # Forward the message to IoT Hub
            message_dict = {
                'WorkStation': "ROS_Cobot", 
                'Status': 9,  # Status code 9 for robot feedback
                'Feedback': msg.data
            }
            
            self.send_message_to_iot_hub(message_dict)
            
        except Exception as e:
            self.get_logger().error(f"Error processing robot feedback: {e}")
    
    def send_status_to_iot_hub(self, status, details=""):
        """Send status update to IoT Hub
        
        Args:
            status: Status string (e.g., "Starting", "Complete", "Error")
            details: Additional details about the status
        """
        message_dict = {
            'WorkStation': "ROS_Cobot",
            'Status': self._get_status_code(status),
            'Feedback': details
        }
        
        self.send_message_to_iot_hub(message_dict)
    
    def _get_status_code(self, status):
        """Convert status string to numeric code for IoT Hub
        
        Args:
            status: Status string
            
        Returns:
            int: Status code
        """
        status_codes = {
            "Starting": 1,
            "InProgress": 2,
            "Complete": 3,
            "Error": 4,
            "Feedback": 9
        }
        
        return status_codes.get(status, 0)
    
    def send_message_to_iot_hub(self, message_dict):
        """Send a message to IoT Hub
        
        Args:
            message_dict: Dictionary containing the message data
        """
        try:
            # Convert dict to JSON string
            message_json = json.dumps(message_dict)
            
            # Create and send the message
            message = Message(message_json)
            self.iot_client.send_message(message)
            self.get_logger().info(f"Message sent to IoT Hub: {message_json}")
            
        except Exception as e:
            self.get_logger().error(f"Error sending message to IoT Hub: {e}")
    
    def destroy_node(self):
        """Clean up resources when the node is destroyed"""
        # Terminate any running process
        if self.current_process and self.current_process.poll() is None:
            self.current_process.terminate()
            self.current_process.wait()
        
        # Disconnect from IoT Hub
        if self.iot_client:
            self.iot_client.shutdown()
            self.get_logger().info("Disconnected from IoT Hub")
        
        super().destroy_node()

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create the node
    ros2_azure_bridge = ROS2AzureIoTBridge()
    
    try:
        # Spin the node to process callbacks
        rclpy.spin(ros2_azure_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        ros2_azure_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
