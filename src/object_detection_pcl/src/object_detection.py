#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import pcl
import pcl.pcl_visualization
from pcl import PointCloud
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Header

class PointCloudCircleDetector(Node):

    def __init__(self):
        super().__init__('point_cloud_circle_detector')

        # Subscriber to the point cloud data
        self.subscription = self.create_subscription(
            PointCloud2,
            '/UR5/UR5_camera/depth/color/points',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('PointCloud2 message received')

        # Convert PointCloud2 message to numpy array
        cloud_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        cloud_np = np.array(list(cloud_data))
        
        # Create a pcl PointCloud from the numpy array
        cloud = pcl.PointCloud()
        cloud.from_array(cloud_np.astype(np.float32))

        # Apply radius outlier removal filter to clean the point cloud
        sor = cloud.make_radius_outlier_removal()
        sor.set_radius_search(0.01)  # Radius in meters to define a neighborhood
        sor.set_min_neighbors_in_radius(5)  # Minimum neighbors in radius to be considered a valid point
        cloud_filtered = sor.filter()

        # Use sample consensus to detect circles
        seg = cloud_filtered.make_segmenter()
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_CIRCLE3D)  # 3D circle model
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.01)  # Maximum distance from model to a point to be considered inlier

        # Call segment function to obtain set of inlier indices and model coefficients
        inlier_indices, model = seg.segment()

        # If any circles were found, print and visualize them
        if len(inlier_indices) > 0:
            self.get_logger().info(f"Detected circle with center: {model[0]}, radius: {model[1]}")

            # Visualization (optional) using pcl visualization library
            vis = pcl.pcl_visualization.CloudViewing()
            vis.DrawGeometries([cloud_filtered])

        else:
            self.get_logger().warn("No circles detected in the point cloud.")

def main(args=None):
    rclpy.init(args=args)
    point_cloud_circle_detector = PointCloudCircleDetector()
    rclpy.spin(point_cloud_circle_detector)
    point_cloud_circle_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

