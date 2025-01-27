#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
import re
# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)

class ObjectDetectionListener(Node):
	def __init__(self):
		super().__init__('object_detection_listener')

		# Subscribe to the detected object topic
		self.subscription = self.create_subscription(
			String,
			'/detected_object',
			self.detection_callback,
			10
		)
		self.scene = MoveItPy(node_name="planning_scene")
		self.robot = self.scene.get_planning_component("ur5_arm")
		planning_scene_monitor = self.scene.get_planning_scene_monitor()
		# Initialize the MoveIt! planning scene interface
	   

		# Camera parameters (example for RealSense D435i)
		# You need to replace these with actual camera intrinsics if available
		self.fx = 616.1530  # Focal length in x direction (in pixels)
		self.fy = 616.1530  # Focal length in y direction (in pixels)
		self.cx = 320.0  # Principal point (x)
		self.cy = 240.0  # Principal point (y)

		self.get_logger().info('Ready to listen to detections and place objects in RViz.')

	def detection_callback(self, msg):
		try:
			# Parse the message to extract label, center coordinates, and depth value
			detection_data = msg.data
			self.get_logger().info(f"Received detection data: {detection_data}")

			# Search for the label "Cup" in the message
			if "Cup" in detection_data:
				# Extract the center_x, center_y, and depth value from the message
				match = re.search(r"Label: (.+?) \| Center: \((\d+), (\d+)\) \| Depth: ([\d\.]+)m", detection_data)
				if match:
					label = match.group(1)
					center_x = int(match.group(2))
					center_y = int(match.group(3))
					depth = float(match.group(4))  # Depth in meters

					self.get_logger().info(f"Found a Cup at ({center_x}, {center_y}) with depth {depth} meters.")

					# Convert the 2D coordinates to 3D coordinates
					x = (center_x - self.cx) * depth / self.fx
					y = (center_y - self.cy) * depth / self.fy
					z = depth  # Z is simply the depth value in meters

					# Create a pose for the object (a box in the 3D space)
					self.place_box_in_rviz(x, y, z)

		except Exception as e:
			self.get_logger().error(f"Error processing detection data: {e}")

	def place_box_in_rviz(self, x, y, z):
		with planning_scene_monitor.read_write() as scene:
			collision_object = CollisionObject()
			collision_object.header.frame_id = "UR5_camera_depth_optical_frame"
			collision_object.id = "boxes"

			box_pose = Pose()
			box_pose.position.x = x
			box_pose.position.y = y
			box_pose.position.z = z

			box = SolidPrimitive()
			box.type = SolidPrimitive.BOX
			box.dimensions = dimensions

			collision_object.primitives.append(box)
			collision_object.primitive_poses.append(box_pose)
			collision_object.operation = CollisionObject.ADD

			scene.apply_collision_object(collision_object)
			scene.current_state.update()  # Important to ensure the scene is updated
			self.get_logger().info(f"Placed a box at 3D coordinates: ({x}, {y}, {z})")

def main(args=None):
	rclpy.init(args=args)
	detection_listener = ObjectDetectionListener()
	rclpy.spin(detection_listener)
	detection_listener.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

