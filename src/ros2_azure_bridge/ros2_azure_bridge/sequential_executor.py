#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading
import time
from rclpy.executors import SingleThreadedExecutor

class CommandSequencer(Node):
    def __init__(self):
        super().__init__('command_sequencer')
        self.publisher = self.create_publisher(String, '/UR5/cloud_messages', 10)
        self.subscription = self.create_subscription(String, '/UR5/cloud_messages', self.message_callback, 10)
        
        # Commands to be executed in sequence
        self.commands = [
            "ros2 run edit_planning_scene add_object_to_planning_scene_node",
            "ros2 launch ur5_moveit_test ur5_move_to_camera.launch.py",
            "ros2 run object_detection semantic_segmentation_cup.py",
            "ros2 run point_cloud_circle_detector plantpot_pointcloud_segmentation_and_coloring_service",
            "ros2 run point_cloud_circle_detector plantpot_placement_service",
            "ros2 launch ur5_moveit_test ur5_image_octmap_path.launch.py",
            "ros2 launch ur5_moveit_test ur5_pickup_sensor.launch.py",
            "ros2 launch ur5_moveit_test ur5_probe_sensor_service.launch.py",
            "ros2 launch ur5_moveit_test ur5_move_plantpot_to_home.launch.py",
            "ros2 launch ur5_moveit_test ur5_drop_sensor.launch.py"
        ]
        
        self.current_command_index = 0
        self.waiting_for_confirmation = False
        self.current_process = None
        self.cleanup_delay = 3.0  # Delay after process termination
        self.startup_delay = 2.0  # Delay before starting next command
        
        self.timer = self.create_timer(2.0, self.start_execution)
        self.get_logger().info('Command sequencer initialized. Starting execution in 2 seconds...')

    def start_execution(self):
        self.timer.cancel()
        self.execute_next_command()

    def execute_next_command(self):
        if self.current_command_index >= len(self.commands):
            self.get_logger().info('All commands have been executed!')
            return

        command = self.commands[self.current_command_index]
        self.get_logger().info(f'Executing command [{self.current_command_index + 1}/{len(self.commands)}]: {command}')
        thread = threading.Thread(target=self.run_command, args=(command,))
        thread.daemon = True
        thread.start()

    def run_command(self, command):
        try:
            self.waiting_for_confirmation = True
            self.current_process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            while self.waiting_for_confirmation:
                # Non-blocking wait; process is running
                time.sleep(2)

            # After confirmation, kill the process if still running
            if self.current_process and self.current_process.poll() is None:
                self.get_logger().info(f'Terminating process for: {command}')
                self.current_process.terminate()
                
                # Give the process time to terminate gracefully
                try:
                    self.current_process.wait(timeout=5.0)
                    self.get_logger().info(f'Process terminated gracefully.')
                except subprocess.TimeoutExpired:
                    self.get_logger().warn(f'Process did not terminate gracefully, forcing kill.')
                    self.current_process.kill()
                    self.current_process.wait()

            msg = String()
            msg.data = f"TERMINATED: {command}"
            self.publisher.publish(msg)
            self.get_logger().info(f'Published termination message: {msg.data}')

            self.current_process = None
            
            # Wait for cleanup delay to ensure proper resource cleanup
            self.get_logger().info(f'Waiting {self.cleanup_delay} seconds for cleanup...')
            time.sleep(self.cleanup_delay)

        except Exception as e:
            self.get_logger().error(f'Exception occurred: {str(e)}')
            msg = String()
            msg.data = f"EXCEPTION: {command} - {str(e)}"
            self.publisher.publish(msg)

        # Additional delay before starting next command to prevent overlap
        self.get_logger().info(f'Waiting {self.startup_delay} seconds before starting next command...')
        time.sleep(self.startup_delay)

        # Move to next command after all delays
        self.current_command_index += 1
        self.execute_next_command()

    def message_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')
        
        # Ignore messages that this node itself publishes
        if msg.data.startswith("TERMINATED:") or msg.data.startswith("EXCEPTION:"):
            return

        if self.waiting_for_confirmation:
            self.waiting_for_confirmation = False
            self.get_logger().info('Confirmation received. Terminating current process and moving on.')	

def main(args=None):
    rclpy.init(args=args)
    
    command_sequencer = CommandSequencer()
    
    executor = SingleThreadedExecutor()
    executor.add_node(command_sequencer)
    
    try:
        print('Starting executor. Press Ctrl+C to terminate.')
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        command_sequencer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
