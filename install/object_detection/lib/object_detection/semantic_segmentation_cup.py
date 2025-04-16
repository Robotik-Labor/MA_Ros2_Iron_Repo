#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String
from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as PILImage
import torch
import time

from transformers import AutoProcessor, AutoModelForUniversalSegmentation

class OneFormerCupDetectionNode(Node):
    def __init__(self):
        super().__init__('oneformer_cup_detection_node')
        
        # Disable torch deprecation warnings
        torch.set_warn_always(False)
        
        # ROS parameters
        self.declare_parameter('input_topic', '/UR5/UR5_camera/color/image_raw')
        self.declare_parameter('semantic_topic', '/UR5/semantic_segmentation')
        self.declare_parameter('cup_coordinates_topic', '/UR5/cup_pixel_coordinates')
        
        # Get parameters
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        semantic_topic = self.get_parameter('semantic_topic').get_parameter_value().string_value
        cup_coordinates_topic = self.get_parameter('cup_coordinates_topic').get_parameter_value().string_value
        
        # Subscribers and Publishers
        self.image_subscription = self.create_subscription(
            ROSImage, 
            input_topic, 
            self.image_callback, 
            1
        )
        
        self.cloud_message_publisher = self.create_publisher(
            String, 
            '/UR5/cloud_messages', 
            10
        )
        
        
        # Create publishers for semantic segmentation and cup coordinates
        self.semantic_publisher = self.create_publisher(ROSImage, semantic_topic, 10)
        self.cup_coordinates_publisher = self.create_publisher(Float32MultiArray, cup_coordinates_topic, 10)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Load OneFormer model
        self.get_logger().info('Loading OneFormer model...')
        try:
            with torch.no_grad():
                # Use AutoProcessor and AutoModel
                self.processor = AutoProcessor.from_pretrained("shi-labs/oneformer_coco_swin_large")
                self.model = AutoModelForUniversalSegmentation.from_pretrained("shi-labs/oneformer_coco_swin_large")
                
                # Move model to GPU if available
                self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
                self.model.to(self.device)
                self.model.eval()  # Set model to evaluation mode
            
            # COCO dataset typically uses index 41 for cup 58 for potted plants and 75 for vase
            
            self.cup_class_index = 58
            
            self.get_logger().info(f'OneFormer Cup Detection Node initialized on {self.device}')
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
        cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
        # Convert to PIL Image
        pil_image = PILImage.fromarray(cv_image)
        
        try:
            # Use explicit indexing for meshgrid
            with torch.no_grad():
                # Semantic Segmentation
                semantic_inputs = self.processor(images=pil_image, task_inputs=["semantic"], return_tensors="pt")
                semantic_inputs = {k: v.to(self.device) for k, v in semantic_inputs.items()}
                semantic_outputs = self.model(**semantic_inputs)
                
                # Post-process semantic segmentation
                predicted_semantic_map = self.processor.post_process_semantic_segmentation(
                    semantic_outputs, 
                    target_sizes=[pil_image.size[::-1]]
                )[0]
                
                # Convert to numpy for processing
                semantic_map_np = predicted_semantic_map.numpy()

                
                # Publish semantic segmentation visualization
                semantic_img = self.convert_to_ros_image(semantic_map_np, msg.header)
                self.semantic_publisher.publish(semantic_img)
                
                # Find and process cup pixels
                cup_pixels = self.find_cup_pixels(semantic_map_np)
                
                # Publish cup pixel coordinates
                if len(cup_pixels) > 0:
                    self.publish_cup_coordinates(cup_pixels)
            
        except Exception as e:
            self.get_logger().error(f'Segmentation error: {str(e)}')

    def find_cup_pixels(self, semantic_map):
        """
        Find pixels corresponding to cups and group them
        
        Args:
            semantic_map (numpy.ndarray): Semantic segmentation map
        
        Returns:
            list: List of cup pixel groups
        """
        # Find pixels with cup class index
        cup_mask = (semantic_map == self.cup_class_index)
        
        # Find connected components (groups of cup pixels)
        num_labels, labels_map, stats, centroids = cv2.connectedComponentsWithStats(
            cup_mask.astype(np.uint8), connectivity=8
        )
        
        # Store cup pixel groups
        cup_pixel_groups = []
        
        # Skip the first label (background)
        for i in range(1, num_labels):
            # Only consider components with significant area
            if stats[i, cv2.CC_STAT_AREA] > 50:  # Minimum 50 pixels
                # Get pixel coordinates for this cup group
                group_pixels = np.column_stack(np.where(labels_map == i))
                
                cup_pixel_groups.append({
                    'pixels': group_pixels,
                    'centroid': centroids[i],
                    'area': stats[i, cv2.CC_STAT_AREA]
                })
        
        return cup_pixel_groups
    
    def publish_cup_coordinates(self, cup_pixel_groups):
        """
        Publish cup pixel coordinates with correct ROS message layout
        
        Args:
            cup_pixel_groups (list): List of cup pixel groups
        """
        if not cup_pixel_groups:
            return
        
        # Create a Float32MultiArray message
        coords_msg = Float32MultiArray()
        
        # Prepare dimensions
        
        # Initialize the list to store the pixel data in the desired format
        all_pixels = []

        # Process the groups
        for group_index, group in enumerate(cup_pixel_groups):
            # Store the group number, followed by the pixel coordinates (x, y)
            for pixel in group['pixels']:
                # The format will be: Group, pixel_x, pixel_y
                all_pixels.extend([float(group_index), float(pixel[1]), float(pixel[0])])

        # Assign the flattened list of pixels to the message data
        coords_msg.data = all_pixels

        # Publish the message        # Initialize the list to store the pixel data in the desired format
        all_pixels = []
        total_pixel_count = 0

        # Process the groups
        for group_index, group in enumerate(cup_pixel_groups):
            # Store the group number, followed by the pixel coordinates (x, y)
            for pixel in group['pixels']:
                # The format will be: Group, pixel_x, pixel_y
                all_pixels.extend([float(group_index), float(pixel[1]), float(pixel[0])])
                total_pixel_count += 1

        # Assign the flattened list of pixels to the message data
        # Set dimensions correctly
        group_dim = MultiArrayDimension()
        group_dim.label = "groups"
        group_dim.size = len(cup_pixel_groups)
        group_dim.stride = 1
        pixel_count_dim = MultiArrayDimension()
        pixel_count_dim.label = "pixels"
        pixel_count_dim.size = total_pixel_count
        pixel_count_dim.stride = total_pixel_count * 2
        
        coords_msg.layout.dim = [group_dim, pixel_count_dim]     

        coords_msg.data = all_pixels

        # Publish the message
        self.cup_coordinates_publisher.publish(coords_msg)

        # Log detected cup group information
        self.get_logger().info(
            f"Detected {len(cup_pixel_groups)} Cup Groups: " + 
            ", ".join([f"Group {i+1}: {len(group['pixels'])} pixels" for i, group in enumerate(cup_pixel_groups)])
        )
        
        msg = String()
        msg.data = "Segementation_Complete"
        self.cloud_message_publisher.publish(msg)
        
        
        time.sleep(1)

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
    node = OneFormerCupDetectionNode()
    
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down OneFormer Cup Detection Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
