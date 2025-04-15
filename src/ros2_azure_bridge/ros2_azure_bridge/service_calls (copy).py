#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from common_services_package.srv import GetPlantpotCoords  # Import your custom service
from std_srvs.srv import Trigger
from threading import Lock


class PlantpotCoordCacheService(Node):
    def __init__(self):
        super().__init__('plantpot_coord_cache_service')
        
        # Create a lock for thread safety
        self._lock = Lock()
        
        # Initialize last coordinates storage
        self._last_coords = Point()
        self._has_received_message = False
        
        # Create subscription to the plantpot coordinates topic
        self.subscription = self.create_subscription(
            Point,
            '/UR5/plantpot_coords',
            self.coord_callback,
            10)
            
        # Create service to retrieve the last coordinates
        self.service = self.create_service(
            GetPlantpotCoords,
            '/UR5/service/plantpot_coords',
            self.get_last_coords_callback)
            
        self.get_logger().info('Plantpot coordinates cache service started')

    def coord_callback(self, msg):
        with self._lock:
            self._last_coords = msg
            self._has_received_message = True
            self.get_logger().info(f'Cached coordinates: x={msg.x}, y={msg.y}, z={msg.z}')

    def get_last_coords_callback(self, request, response):
        with self._lock:
            if self._has_received_message:
                response.success = True
                response.message = f"x: {self._last_coords.x}, y: {self._last_coords.y}, z: {self._last_coords.z}"
                response.coordinates = self._last_coords
                self.get_logger().info(f'Returning cached coordinates: {response.message}')
            else:
                response.success = False
                response.message = "No coordinates have been received yet"
                self.get_logger().info('No coordinates available to return')
        return response


def main(args=None):
    rclpy.init(args=args)
    plantpot_coord_cache_service = PlantpotCoordCacheService()
    
    try:
        rclpy.spin(plantpot_coord_cache_service)
    except KeyboardInterrupt:
        pass
    finally:
        plantpot_coord_cache_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
