#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as PILImage
import torch

from transformers import AutoProcessor, AutoModelForUniversalSegmentation

class OneFormerSegmentationNode(Node):
    def __init__(self):
        super().__init__('oneformer_segmentation_node')
        
        # Disable torch deprecation warnings
        torch.set_warn_always(False)
        
        # ROS parameters
        self.declare_parameter('input_topic', '/UR5/UR5_camera/color/image_raw')
        self.declare_parameter('semantic_topic', '/semantic_segmentation')
        self.declare_parameter('instance_topic', '/instance_segmentation')
        self.declare_parameter('panoptic_topic', '/panoptic_segmentation')
        
        # Get parameters
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        semantic_topic = self.get_parameter('semantic_topic').get_parameter_value().string_value
        instance_topic = self.get_parameter('instance_topic').get_parameter_value().string_value
        panoptic_topic = self.get_parameter('panoptic_topic').get_parameter_value().string_value
        
        # Subscribers and Publishers
        self.image_subscription = self.create_subscription(
            ROSImage, 
            input_topic, 
            self.image_callback, 
            10
        )
        
        # Create publishers for different segmentation types
        self.semantic_publisher = self.create_publisher(ROSImage, semantic_topic, 10)
        self.instance_publisher = self.create_publisher(ROSImage, instance_topic, 10)
        self.panoptic_publisher = self.create_publisher(ROSImage, panoptic_topic, 10)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Load OneFormer model with explicit indexing
        self.get_logger().info('Loading OneFormer model...')
        try:
            with torch.no_grad():
                # Use AutoProcessor and AutoModel to avoid deprecation warnings
                self.processor = AutoProcessor.from_pretrained("shi-labs/oneformer_coco_swin_large")
                self.model = AutoModelForUniversalSegmentation.from_pretrained("shi-labs/oneformer_coco_swin_large")
                
                # Move model to GPU if available
                self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
                self.model.to(self.device)
                self.model.eval()  # Set model to evaluation mode
            
            self.get_logger().info(f'OneFormer Segmentation Node initialized on {self.device}')
        except Exception as e:
            self.get_logger().error(f'Model loading error: {str(e)}')
            raise

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {str(e)}')
            return
        
        # Convert to PIL Image
        pil_image = PILImage.fromarray(cv_image)
        
        try:
            # Use explicit indexing for meshgrid
            with torch.no_grad():
                # Semantic Segmentation
                semantic_inputs = self.processor(images=pil_image, task_inputs=["semantic"], return_tensors="pt")
                semantic_inputs = {k: v.to(self.device) for k, v in semantic_inputs.items()}
                semantic_outputs = self.model(**semantic_inputs)
                predicted_semantic_map = self.processor.post_process_semantic_segmentation(
                    semantic_outputs, 
                    target_sizes=[pil_image.size[::-1]]
                )[0]
                semantic_img = self.convert_to_ros_image(predicted_semantic_map.numpy(), msg.header)
                self.semantic_publisher.publish(semantic_img)
                
                # Instance Segmentation
                instance_inputs = self.processor(images=pil_image, task_inputs=["instance"], return_tensors="pt")
                instance_inputs = {k: v.to(self.device) for k, v in instance_inputs.items()}
                instance_outputs = self.model(**instance_inputs)
                predicted_instance_map = self.processor.post_process_instance_segmentation(
                    instance_outputs, 
                    target_sizes=[pil_image.size[::-1]]
                )[0]["segmentation"]
                instance_img = self.convert_to_ros_image(predicted_instance_map.numpy(), msg.header)
                self.instance_publisher.publish(instance_img)
                
                # Panoptic Segmentation
                panoptic_inputs = self.processor(images=pil_image, task_inputs=["panoptic"], return_tensors="pt")
                panoptic_inputs = {k: v.to(self.device) for k, v in panoptic_inputs.items()}
                panoptic_outputs = self.model(**panoptic_inputs)
                predicted_panoptic_map = self.processor.post_process_panoptic_segmentation(
                    panoptic_outputs, 
                    target_sizes=[pil_image.size[::-1]]
                )[0]["segmentation"]
                panoptic_img = self.convert_to_ros_image(predicted_panoptic_map.numpy(), msg.header)
                self.panoptic_publisher.publish(panoptic_img)
            
        except Exception as e:
            self.get_logger().error(f'Segmentation error: {str(e)}')

    def convert_to_ros_image(self, segmentation_map, header):
        """
        Convert segmentation map to ROS Image message
        """
        # Normalize the segmentation map for visualization
        if segmentation_map.ndim == 2:
            segmentation_map = self.colorize_segmentation_map(segmentation_map)
        
        # Convert to ROS image message
        ros_image = self.bridge.cv2_to_imgmsg(segmentation_map, encoding="rgb8")
        ros_image.header = header
        return ros_image

    def colorize_segmentation_map(self, segmentation_map):
        """
        Colorize the segmentation map for visualization
        """
        # Create a color palette (you can customize this)
        palette = np.random.randint(0, 255, (256, 3), dtype=np.uint8)
        
        # Map segmentation indices to colors
        colored_map = palette[segmentation_map]
        
        return colored_map

def main(args=None):
    rclpy.init(args=args)
    node = OneFormerSegmentationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down OneFormer Segmentation Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()