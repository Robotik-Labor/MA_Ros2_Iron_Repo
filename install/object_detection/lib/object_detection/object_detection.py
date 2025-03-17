#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import tensorflow as tf
import numpy as np

class ImageSubscriber(Node):
	def __init__(self):
		super().__init__('image_subscriber_node')
		self.subscription = self.create_subscription(
			Image,
			'/UR5/UR5_camera/color/image_raw',  # Change this to the appropriate topic name
			self.image_callback,
			10
		)
		self.bridge = CvBridge()
		
	
		# Load a pre-trained image classification model (MobileNetV2)
		self.model = tf.keras.applications.MobileNetV2(weights='imagenet')
			
		# Preprocess function for MobileNetV2
		self.preprocess_input = tf.keras.applications.mobilenet_v2.preprocess_input
		self.decode_predictions = tf.keras.applications.mobilenet_v2.decode_predictions


	def image_callback(self, msg):
		try:
			# Convert ROS Image message to OpenCV format
			cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
			self.get_logger().info('Received an image')

			input_image = cv2.resize(cv_image, (224, 224))
				
			# Convert the image to RGB format (MobileNet expects RGB images)
			input_image_rgb = cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB)
				
			# Preprocess the image for the model (this adjusts color and scale)
			input_array = np.expand_dims(input_image_rgb, axis=0)
			input_array = self.preprocess_input(input_array)

			# Run inference with the pre-trained model
			predictions = self.model.predict(input_array)

			# Decode the predictions
			decoded_predictions = self.decode_predictions(predictions, top=1)[0]
				
			# Log the predicted class
			for i, (imagenet_id, label, score) in enumerate(decoded_predictions):
				self.get_logger().info(f"Prediction {i}: {label} with confidence {score:.2f}")
				
			# Assuming we have dynamic bounding boxes (dummy bounding boxes for this example)
			height, width, _ = cv_image.shape

			# Generate dummy bounding boxes and labels (In an actual model, you'd get these from the model's predictions)
			# Format for bounding boxes: [xmin, ymin, xmax, ymax]
			boxes = [[width // 4, height // 4, 3 * width // 4, 3 * height // 4]]  # Example box
			labels = ["Pot Plant"]  # Example label for the box
			scores = [0.95]  # Example confidence score

			# Iterate over all detected boxes (in this case, there's only one box)
			for i, box in enumerate(boxes):
				xmin, ymin, xmax, ymax = box
				label = labels[i]
				score = scores[i]

				# Draw the bounding box (Green color)
				cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

				# Add label and confidence score
				label_text = f"{label} ({score*100:.1f}%)"
				cv2.putText(cv_image, label_text, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)     

			# Display the image using OpenCV (optional, for debugging)
			cv2.imshow("Received Image", cv_image)
			cv2.waitKey(1)
		
		except Exception as e:
			self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
	rclpy.init(args=args)

	# Create the image subscriber node
	image_subscriber = ImageSubscriber()

	# Spin the node so that it can process callbacks
	rclpy.spin(image_subscriber)

	# Clean up and shutdown ROS 2 client library
	image_subscriber.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

