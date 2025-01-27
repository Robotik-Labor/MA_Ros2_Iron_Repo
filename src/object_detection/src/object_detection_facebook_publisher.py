#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # ROS 2 import
from std_msgs.msg import String   # Use String to send detection data
from cv_bridge import CvBridge
import cv2
import torch
from transformers import DetrImageProcessor, DetrForObjectDetection
from PIL import Image as PILImage
import numpy as np

class DetrObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('image_subscriber_node')

        # Initialize depth image variable
        self.depth_image = None

        # Subscriptions
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

        # Publisher to send detection and depth data
        self.detection_publisher = self.create_publisher(
            String,
            '/UR5/detected_objects',  # Topic name to publish to
            10
        )

        # Initialize CvBridge to convert ROS image to OpenCV format
        self.bridge = CvBridge()

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
            #cv_image = cv2.resize(cv_image, (848, 480))  # Resize for DETR model
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

            # Collect results to publish
            detection_data = ""

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

                # Calculate the center of the bounding box
                center_x = (xmin + xmax) // 2
                center_y = (ymin + ymax) // 2

                # Get depth at the center pixel
                depth_value = self.depth_image[center_y, center_x]
                depth_value_meters = depth_value / 1000  # Convert to meters

                # Format the detection data to include label, center coordinates, and depth value
                detection_data += f"Label: {self.model.config.id2label[label.item()]} | "
                detection_data += f"Center: ({center_x}, {center_y}) | "
                detection_data += f"Depth: {depth_value_meters:.2f}m\n"

                # Draw bounding box and labels
                cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                label_text = f"{self.model.config.id2label[label.item()]} ({score.item()*100:.1f}%)"
                cv2.putText(cv_image, label_text, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Publish detection data with label, center, and depth information
            detection_msg = String()
            detection_msg.data = detection_data
            self.detection_publisher.publish(detection_msg)
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

