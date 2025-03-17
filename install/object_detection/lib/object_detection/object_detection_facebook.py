#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # ROS 2 import
from cv_bridge import CvBridge
import cv2
import torch
from transformers import DetrImageProcessor, DetrForObjectDetection
from PIL import Image as PILImage
import numpy as np

class DetrObjectDetectionNode(Node):
	def __init__(self):
		super().__init__('image_subscriber_node')
		self.depth_image = None
		self.depth_subscription = self.create_subscription(
			Image,
			'/UR5/UR5_camera/aligned_depth_to_color/image_raw',  # Change this to the appropriate topic name
			self.depth_callback,
			10
		)
		
		self.subscription = self.create_subscription(
			Image,
			'/UR5/UR5_camera/color/image_raw',  # Change this to the appropriate topic name
			self.image_callback,
			10
		)



		# Initialize CvBridge to convert ROS image to OpenCV format
		self.bridge = CvBridge()
		self.image_received = None

		# Load the DETR model and processor
		self.processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50", revision="no_timm")
		self.model = DetrForObjectDetection.from_pretrained("facebook/detr-resnet-50", revision="no_timm")

	def depth_callback(self, msg):
		try:
			# Convert depth image message to OpenCV format (single channel, 32-bit float)
			self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
		except Exception as e:
			self.get_logger().error(f"Error converting depth image: {e}")

	def image_callback(self, msg):
		try:
			# Convert ROS Image message to OpenCV format
			cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
			self.get_logger().info('Received an image')
			#cv_image = cv2.resize(cv_image, (848, 480))
			# Convert the OpenCV image to PIL Image format
			pil_image = PILImage.fromarray(cv_image)

			# Preprocess the image for the model
			inputs = self.processor(images=pil_image, return_tensors="pt")
			outputs = self.model(**inputs)

			# Post-process the detections
			target_sizes = torch.tensor([pil_image.size[::-1]])  # Convert (width, height) to (height, width)
			results = self.processor.post_process_object_detection(
				outputs, target_sizes=target_sizes, threshold=0.9
			)[0]

			# Iterate over all detected objects
			height, width, _ = cv_image.shape
			for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):

				box = [round(i, 2) for i in box.tolist()]

				# Convert box coordinates to pixels
				xmin, ymin, xmax, ymax = box
				xmin = int(xmin)
				ymin = int(ymin)
				xmax = int(xmax)
				ymax = int(ymax)

				#height, width = self.depth_image.shape
				#self.get_logger().info("Depth_Image")				
				#self.get_logger().info(str(height))
				#self.get_logger().info(str(width))


				center_x = (xmin + xmax) // 2
				center_y = (ymin + ymax) // 2

				# Get depth at the center pixel
				depth_value = self.depth_image[center_y, center_x]

				# You can now use the depth_value as the distance (in meters) to the object
				self.get_logger().info(f"Depth at ({center_x}, {center_y}) is {depth_value/1000} meters")

				# Draw depth value on the image
				depth_text = f"{depth_value/1000}m"
				cv2.putText(cv_image, depth_text, (xmin, ymin - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

				# Get the label text (convert label index to string)
				label_text = self.model.config.id2label[label.item()]
				confidence = round(score.item(), 3)
				self.get_logger().info(label_text)
				self.get_logger().info(str(confidence))
				self.get_logger().info(str(box))
				self.get_logger().info(str(height))
				self.get_logger().info(str(width))
				# Draw the bounding box
				cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

				# Add label and confidence score
				label_text = f"{label_text} ({confidence*100:.1f}%)"
				cv2.putText(cv_image, label_text, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

			# Display the image using OpenCV
			cv2.imshow("Detected Objects", cv_image)
			cv2.waitKey(1)

		except Exception as e:
			self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
	rclpy.init(args=args)

	# Create the object detection node
	detr_node = DetrObjectDetectionNode()

	# Spin the node so that it can process callbacks
	rclpy.spin(detr_node)

	# Clean up and shutdown ROS 2 client library
	detr_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

