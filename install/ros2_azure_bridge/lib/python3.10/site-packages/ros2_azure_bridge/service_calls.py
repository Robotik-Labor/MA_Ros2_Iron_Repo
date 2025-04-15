#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
from common_services_package.srv import GetPlantpotCoords  # Original service
from common_services_package.srv import GetFloat32Array    # New service (you'll need to create this)
from common_services_package.srv import GetPointCloud2     # New service (you'll need to create this)
from std_srvs.srv import Trigger
from threading import Lock

class ExtendedCoordCacheService(Node):
    def __init__(self):
        super().__init__('extended_coord_cache_service')
        
        # Create locks for thread safety
        self._point_lock = Lock()
        self._array_lock = Lock()
        self._cloud_lock = Lock()
        
        # Initialize storage for all message types
        self._last_coords = Point()
        self._has_received_point = False
        
        self._last_array = Float32MultiArray()
        self._has_received_array = False
        
        self._last_pointcloud = PointCloud2()
        self._has_received_pointcloud = False
        
        # Create subscriptions to all topics
        self.point_subscription = self.create_subscription(
            Point,
            '/UR5/plantpot_coords',
            self.point_callback,
            10)
            
        self.array_subscription = self.create_subscription(
            Float32MultiArray,
            '/UR5/cup_pixel_coordinates',  # Replace with your actual topic name
            self.array_callback,
            10)
            
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/UR5/plantpod_reduced_pointcloud',  # Replace with your actual topic name
            self.pointcloud_callback,
            10)
            
        # Create services to retrieve the last messages
        self.point_service = self.create_service(
            GetPlantpotCoords,
            '/UR5/service/plantpot_coords',
            self.get_last_point_callback)
            
        self.array_service = self.create_service(
            GetFloat32Array,
            '/UR5/service/cup_pixel_coordinates',
            self.get_last_array_callback)
            
        self.pointcloud_service = self.create_service(
            GetPointCloud2,
            '/UR5/service/pointcloud_data',
            self.get_last_pointcloud_callback)
            
        self.get_logger().info('Extended coordinates cache service started')
    
    # Callback functions for all subscriptions
    def point_callback(self, msg):
        with self._point_lock:
            self._last_coords = msg
            self._has_received_point = True
            self.get_logger().info(f'Cached point coordinates: x={msg.x}, y={msg.y}, z={msg.z}')
    
    def array_callback(self, msg):
        with self._array_lock:
            self._last_array = msg
            self._has_received_array = True
            self.get_logger().info(f'Cached array data with {len(msg.data)} elements')
    
    def pointcloud_callback(self, msg):
        with self._cloud_lock:
            self._last_pointcloud = msg
            self._has_received_pointcloud = True
            self.get_logger().info(f'Cached pointcloud with {msg.width}x{msg.height} points')
    
    # Service callback functions
    def get_last_point_callback(self, request, response):
        with self._point_lock:
            if self._has_received_point:
                response.success = True
                response.message = f"x: {self._last_coords.x}, y: {self._last_coords.y}, z: {self._last_coords.z}"
                response.coordinates = self._last_coords
                self.get_logger().info(f'Returning cached coordinates: {response.message}')
            else:
                response.success = False
                response.message = "No point coordinates have been received yet"
                self.get_logger().info('No point coordinates available to return')
        return response
    
    def get_last_array_callback(self, request, response):
        with self._array_lock:
            if self._has_received_array:
                response.success = True
                response.message = f"Array with {len(self._last_array.data)} elements"
                response.data = self._last_array
                self.get_logger().info(f'Returning cached array data: {response.message}')
            else:
                response.success = False
                response.message = "No array data has been received yet"
                self.get_logger().info('No array data available to return')
        return response
    
    def get_last_pointcloud_callback(self, request, response):
        with self._cloud_lock:
            if self._has_received_pointcloud:
                response.success = True
                response.message = f"PointCloud with {self._last_pointcloud.width}x{self._last_pointcloud.height} points"
                response.cloud = self._last_pointcloud
                self.get_logger().info(f'Returning cached pointcloud: {response.message}')
            else:
                response.success = False
                response.message = "No pointcloud has been received yet"
                self.get_logger().info('No pointcloud available to return')
        return response

def main(args=None):
    rclpy.init(args=args)
    extended_service = ExtendedCoordCacheService()
    
    try:
        rclpy.spin(extended_service)
    except KeyboardInterrupt:
        pass
    finally:
        extended_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
